#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "hw/i2c/esp32_i2c.h"
#include "hw/irq.h"
#include "hw/xtensa/esp32_clk.h"

#include "../softmmu/simuliface.h"


static uint8_t fifo8_peek(Fifo8 *fifo)
{
    if (fifo->num == 0) abort();
    return fifo->data[fifo->head];
}

static void esp32_i2c_update_irq(Esp32I2CState * s)
{
    int irq_state = !!(s->int_raw_reg & s->int_ena_reg);
    qemu_set_irq(s->irq, irq_state);
}

static void esp32_i2c_do_transaction( void* opaque )
{
    Esp32I2CState* s = Esp32_I2C(opaque);

    uint32_t cmd = s->cmd_reg[s->lastCMD];
    s->lastOpcode = FIELD_EX32( cmd, I2C_CMD, OPCODE );

    //printf("Qemu: esp32_i2c_do_transaction %i\n", s->lastOpcode); fflush( stdout );

    uint64_t time = 0;

    switch( s->lastOpcode )
    {
    case I2C_OPCODE_RSTART:
    {
        // Espelha o proprio comando RSTART (opcode ainda intacto em `cmd`) ANTES de avancar pro
        // proximo -- e' o unico jeito do lado Core (que gera SCL/SDA reais) distinguir "comeca uma
        // transacao nova/repeated-START agora" de "so' mais um byte do mesmo burst de escrita".
        // Sem isto, RSTART era invisivel na arena e o START real (inclusive um REPEATED START no
        // meio de uma transacao, ex: i2c_master_write_read_device()) nunca acontecia no barramento
        // eletrico de verdade -- so' no proprio QEMU, que sempre "via" a transacao completar.
        writeReg( s->iomem.addr+A_I2C_CMD, cmd );
        s->bytesTx = 0;
        s->sr_reg |= 1<<4;                // I2C_BUS_BUSY
        time = 2*s->period_ns/2;

        s->lastCMD++;
        cmd = s->cmd_reg[s->lastCMD];
        s->lastOpcode = FIELD_EX32( cmd, I2C_CMD, OPCODE );
        cmd |= 1<<16; // Mark as start
    } // Fall through

    case I2C_OPCODE_WRITE:
    {
        if( s->bytesTx == 0 ) s->bytesTx = cmd & 0xFF; //FIELD_EX32( cmd, I2C_CMD, BYTE_NUM );
        uint8_t data = fifo8_peek( &s->tx_fifo );
        //printf("Qemu: esp32_i2c CMD write %i %i\n", s->bytesTx, data ); fflush( stdout );
        writeReg( s->iomem.addr+A_I2C_CMD, (cmd & ~0xFF) | data);
        s->int_raw_reg |= 1<<6;          // I2C_BYTE_TRANS
        time += (19*s->period_ns)/2;
    }break;

    case I2C_OPCODE_READ:
    {
        // Espelha o comando READ (opcode + ACK_VAL + BYTE_NUM, todos em `cmd`) pro lado Core, que
        // e' quem de fato clockeia os 8 bits reais em SCL/SDA a partir do escravo endereçado e
        // decide ACK(continua lendo)/NACK(ultimo byte) conforme ACK_VAL -- mesmo papel que o WRITE
        // acima ja tinha pro sentido contrario. O byte realmente recebido so' fica disponivel
        // depois que o timer abaixo disparar esp32_i2c_event() (mesmo pacing de um WRITE: um byte
        // "no barramento" real dura o mesmo tempo em qualquer direcao).
        if( s->bytesTx == 0 ) s->bytesTx = cmd & 0xFF;
        writeReg( s->iomem.addr+A_I2C_CMD, cmd );
        time += (19*s->period_ns)/2;
    }break;

    case I2C_OPCODE_STOP:
    {
        //printf("Qemu: esp32_i2c CMD stop \n" ); fflush( stdout );
        writeReg( s->iomem.addr+A_I2C_CMD, cmd );
        time = (3*s->period_ns)/2;
    }break;

    case I2C_OPCODE_END:
    {
        //printf("Qemu: esp32_i2c CMD end\n\n"); fflush( stdout );
        s->int_raw_reg |= 1<<3;          // END_DETECT
        esp32_i2c_update_irq(s);
    } break;
    default: break;                      // error_report("esp32_i2c: Invalid command %d opcode %d", i_cmd, opcode);
    }

    if( time )
    {
        time += getQemu_ns(); //qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL );
        //printf("Qemu: i2c Event at %i %lu\n", s->number, time); fflush( stdout );
        timer_mod( &s->event_timer, time ); // Time to end byte transaction
    }
}

static void esp32_i2c_event( void* opaque ) // Timer event
{
    Esp32I2CState* s = Esp32_I2C(opaque);

    //printf("Qemu: esp32_i2c_event %i %lu\n", s->lastOpcode, getQemu_ps() ); fflush( stdout );

    // Le' o ACK/NACK REAL que o Core observou no barramento eletrico (bit0 = 1 quando o outro lado
    // NAO puxou SDA pra baixo -- ver Esp32Adapter.cpp::i2cReadRegister, mesmo canal privado
    // Core<->QEMU deste arquivo, nunca visivel do firmware). Antes disto sempre 0 (fixo) -- o
    // ACK_ERR abaixo nunca podia disparar de verdade, com ou sem escravo respondendo.
    uint64_t status = readReg( s->iomem.addr+A_I2C_STATUS );
    uint8_t ackT = (uint8_t)(status & 1u);

    switch( s->lastOpcode )
    {
    case I2C_OPCODE_RSTART:
    {
    }break;

    case I2C_OPCODE_WRITE:
    {
        fifo8_pop( &s->tx_fifo );
        s->bytesTx--;

        s->int_raw_reg &= ~(1<<6);            // Clear I2C_BYTE_TRANS

        s->sr_reg &= ~1;
        s->sr_reg |= ackT;                    // I2C_ACK_REC

        // ACK_ERR real: so' dispara quando o firmware pediu conferencia (ACK_CHECK_EN) E o ACK
        // observado de verdade no barramento (ackT) diverge do que o firmware esperava (ACK_EXP) --
        // antes disto, ackERR vinha so' dos bits que o proprio firmware escreveu (ACK_CHECK_EN/
        // ACK_EXP), nunca comparados contra o ACK real: um escravo que nunca respondesse (endereco
        // errado, dispositivo ausente) nunca gerava ACK_ERR nenhum.
        bool ackERR = false;
        uint32_t cmd = s->cmd_reg[s->lastCMD];

        if( cmd & 1<<8 )                      // ACK_CHECK_EN
        {
            bool ackExpectsNack = (cmd & 1<<9) != 0;  // ACK_EXP: 1 = firmware espera NACK deste byte
            ackERR = (ackT != 0) != ackExpectsNack;
        }

        if( ackERR ){
            s->int_raw_reg |= 1<<10;          // Set ACK_ERR

            printf("Qemu: esp32_i2c_event ackERR\n"); fflush( stdout );
            //return;
        }
        else s->int_raw_reg &= ~(1<<10);      // Clear ACK_ERR
    }break;

    case I2C_OPCODE_READ:
    {
        // Byte que o Core JA' recebeu eletricamente (8 bits clockados + ACK/NACK do MESTRE enviado
        // na janela certa, ver esp32_i2c_do_transaction acima) -- disponivel no mesmo canal privado
        // A_I2C_CMD que o WRITE usa pro sentido contrario.
        uint8_t data = (uint8_t)readReg( s->iomem.addr+A_I2C_CMD );
        if( fifo8_num_free(&s->rx_fifo) == 0 ) error_report("esp32_i2c: RX FIFO overflow");
        else fifo8_push( &s->rx_fifo, data );

        s->bytesTx--;
        s->int_raw_reg |= 1<<6;               // I2C_BYTE_TRANS
    }break;

    case I2C_OPCODE_STOP:
    {
        s->int_raw_reg |= 1<<7;            // TRANS_COMPLETE
        s->sr_reg      &= ~(1<<4);         // Clear I2C_BUS_BUSY
    }break;

    case I2C_OPCODE_END:
    {
        return;
    }
    default: return;                      // error_report("esp32_i2c: Invalid command %d opcode %d", i_cmd, opcode);
    }


    if( (s->lastOpcode == I2C_OPCODE_WRITE || s->lastOpcode == I2C_OPCODE_READ) && s->bytesTx )
    {
        ;
    }
    else{
        s->cmd_reg[s->lastCMD] = FIELD_DP32(s->cmd_reg[s->lastCMD], I2C_CMD, DONE, 1);
        s->lastCMD++;
    }

    esp32_i2c_do_transaction( s );
}

static void esp32_i2c_write_CTR( Esp32I2CState* s, uint16_t newCTR )
{
    if( s->ctr_reg == newCTR ) return;

    //printf("Qemu: esp32_i2c_write_CTR\n"); fflush( stdout );
    writeReg( s->iomem.addr+A_I2C_CTR, newCTR );

    if( newCTR & 1<<5 )               // bit 5: I2C_TRANS_START
    {
        s->lastCMD = 0;
        //printf("\nQemu: esp32_i2c_write_CTR TRANS %i\n", s->bytesTx ); fflush( stdout );
        esp32_i2c_do_transaction( s );

        newCTR &= ~(1<<5);            // Clear I2C_TRANS_START
    }
    s->ctr_reg = newCTR;
}

static uint32_t esp32_i2c_get_status_reg(Esp32I2CState* s)
{
    //uint32_t res = 0;
    //res = FIELD_DP32(res, I2C_STATUS, BUS_BUSY, s->trans_ongoing);

    uint32_t res = s->sr_reg;
    res = FIELD_DP32(res, I2C_STATUS, RXFIFO_CNT, fifo8_num_used(&s->rx_fifo));
    res = FIELD_DP32(res, I2C_STATUS, TXFIFO_CNT, fifo8_num_used(&s->tx_fifo));
    return res;
}

static void esp32_i2c_updt_frequency( Esp32I2CState* s )
{
    uint64_t fAPB = esp32_soc_get_apb_freq();
    uint64_t period_ns = s->low_period_reg+1 + s->high_period_reg+7;
    period_ns = period_ns*1000000000/fAPB;
    //uint32_t fSCL = fAPB / (s->low_period_reg + s->high_period_reg );

    if( s->period_ns == period_ns ) return;
    s->period_ns = period_ns;

    //printf("Qemu: esp32_i2c_updt_frequency: %i %lu, using %s %lu MHz\n", (s->low_period_reg + s->high_period_reg ), period_ns, "APB Clock", fAPB/1000000 );  fflush( stdout );
    writeReg( s->iomem.addr+A_I2C_LOW_PERIOD, period_ns );
}

static uint64_t esp32_i2c_read(void * opaque, hwaddr addr, unsigned int size)
{
    Esp32I2CState * s = Esp32_I2C(opaque);

    switch(addr) {
    case A_I2C_LOW_PERIOD:   return s->low_period_reg;
    case A_I2C_CTR:          return s->ctr_reg;
    case A_I2C_STATUS:       return esp32_i2c_get_status_reg(s);
    case A_I2C_TIMEOUT:      return s->timeout_reg;
    case A_I2C_FIFO_DATA: {
        if( fifo8_num_used(&s->rx_fifo) == 0) {
            error_report("esp32_i2c: read I2C FIFO while it is empty");
            return 0xee;
        }
        uint8_t res = fifo8_pop(&s->rx_fifo);
        return res;
    }
    case A_I2C_INT_RAW:      return s->int_raw_reg;
    case A_I2C_INT_ENA:      return s->int_ena_reg;
    case A_I2C_INT_ST:       return s->int_raw_reg & s->int_ena_reg;
    case A_I2C_CMD ... (A_I2C_CMD + ESP32_I2C_CMD_COUNT * 4):
        return s->cmd_reg[(addr - A_I2C_CMD) / 4];
    case A_I2C_SDA_HOLD:     return s->sda_hold_reg;
    case A_I2C_SDA_SAMPLE:   return s->sda_sample_reg;
    case A_I2C_HIGH_PERIOD:  return s->high_period_reg;
    case A_I2C_START_HOLD:   return s->start_hold_reg;
    case A_I2C_RSTART_SETUP: return s->rstart_setup_reg;
    case A_I2C_STOP_HOLD:    return s->stop_hold_reg;
    case A_I2C_STOP_SETUP:   return s->stop_setup_reg;
    default:                 return 0;
    }
}

static void esp32_i2c_write(void * opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    Esp32I2CState * s = Esp32_I2C(opaque);

    switch(addr) {
    case A_I2C_LOW_PERIOD:   s->low_period_reg   = value; esp32_i2c_updt_frequency( s ); break;
    case A_I2C_CTR:          esp32_i2c_write_CTR( s, value ); break;
    case A_I2C_TIMEOUT:      s->timeout_reg      = value; break;
    case A_I2C_FIFO_CONF:
        if( FIELD_EX32(value, I2C_FIFO_CONF, NONFIFO_EN) ) error_report("esp32_i2c: APB mode not implemented");
        if( FIELD_EX32(value, I2C_FIFO_CONF, RX_FIFO_RST)) fifo8_reset(&s->rx_fifo);
        if( FIELD_EX32(value, I2C_FIFO_CONF, TX_FIFO_RST)) fifo8_reset(&s->tx_fifo);
        break;
    case A_I2C_FIFO_DATA:
        if( fifo8_num_free(&s->tx_fifo) == 0) error_report("esp32_i2c: write to I2C TX FIFO while it is full");
        else {
            fifo8_push(&s->tx_fifo, value);
            //printf("fifo8_push %lu\n", value);
        }
        break;
    case A_I2C_INT_CLR: s->int_raw_reg &= ~value; esp32_i2c_update_irq(s); break;
    case A_I2C_INT_ENA: s->int_ena_reg  =  value; esp32_i2c_update_irq(s); break;
    case A_I2C_CMD ... (A_I2C_CMD + ESP32_I2C_CMD_COUNT * 4):
        s->cmd_reg[(addr - A_I2C_CMD) / 4] = value;
        break;
    case A_I2C_SDA_HOLD:     s->sda_hold_reg     = value; break;
    case A_I2C_SDA_SAMPLE:   s->sda_sample_reg   = value; break;
    case A_I2C_HIGH_PERIOD:  s->high_period_reg  = value; esp32_i2c_updt_frequency( s ); break;
    case A_I2C_START_HOLD:   s->start_hold_reg   = value; break;
    case A_I2C_RSTART_SETUP: s->rstart_setup_reg = value; break;
    case A_I2C_STOP_HOLD:    s->stop_hold_reg    = value; break;
    case A_I2C_STOP_SETUP:   s->stop_setup_reg   = value; break;
    default:  break;
    }
}

//static void esp32_i2c_do_transaction(Esp32I2CState* s)
//{
//    bool stop_or_end = false;
//
//    for( int i_cmd=0; i_cmd<ESP32_I2C_CMD_COUNT && !stop_or_end; ++i_cmd )
//    {
//        uint32_t cmd = s->cmd_reg[i_cmd];
//        char opcode = FIELD_EX32(cmd, I2C_CMD, OPCODE);
//        switch (opcode) {
//            case I2C_OPCODE_RSTART:
//                i2c_end_transfer(s->bus);
//                s->trans_ongoing = false;
//                break;
//            case I2C_OPCODE_WRITE: {
//                size_t length = FIELD_EX32(cmd, I2C_CMD, BYTE_NUM);
//                if( !s->trans_ongoing) {
//                    s->trans_ongoing = true;
//                    uint8_t data = fifo8_pop(&s->tx_fifo);
//                    uint8_t addr = data >> 1;
//                    uint8_t is_read = data & 0x1;
//                    if( i2c_start_transfer(s->bus, addr, is_read) != 0) {
//                        /* NACK */
//                        if( FIELD_EX32(cmd, I2C_CMD, ACK_CHECK_EN)
//                            && FIELD_EX32(cmd, I2C_CMD, ACK_EXP) == 0) {
//                            s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW, ACK_ERR, 1);
//                            stop_or_end = true;
//                        }
//                        s->trans_ongoing = false;
//                        break;
//                    }
//                    s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW, ACK_ERR, 0);
//                    length -= 1;
//                }
//                for( size_t nbytes = 0; nbytes < length; ++nbytes) {
//                    uint8_t data = fifo8_pop(&s->tx_fifo);
//                    i2c_send(s->bus, data);
//                }
//                break;
//            }
//            case I2C_OPCODE_READ: {
//                size_t length = FIELD_EX32(cmd, I2C_CMD, BYTE_NUM);
//                for( size_t nbytes = 0; nbytes < length; ++nbytes) {
//                    if( fifo8_num_free(&s->rx_fifo) == 0) {
//                        error_report("esp32_i2c: RX FIFO overflow");
//                    } else {
//                        uint8_t data = i2c_recv(s->bus);
//                        fifo8_push(&s->rx_fifo, data);
//                    }
//                }
//                break;
//            }
//            case I2C_OPCODE_STOP:
//                i2c_end_transfer(s->bus);
//                s->trans_ongoing = false;
//                s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW, TRANS_COMPLETE, 1);
//                stop_or_end = true;
//                break;
//            case I2C_OPCODE_END:
//                s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW, END_DETECT, 1);
//                stop_or_end = true;
//                break;
//            default:
//                error_report("esp32_i2c: Invalid command %d opcode %d", i_cmd, opcode);
//                break;
//        }
//        s->cmd_reg[i_cmd] = FIELD_DP32(s->cmd_reg[i_cmd], I2C_CMD, DONE, 1);
//    }
//    esp32_i2c_update_irq(s);
//}

static void esp32_i2c_reset(DeviceState * dev)
{
    Esp32I2CState * s = Esp32_I2C(dev);

    fifo8_reset(&s->rx_fifo);
    fifo8_reset(&s->tx_fifo);
    s->period_ns = 0;
    s->trans_ongoing = false;
    s->ctr_reg = 0;
    s->timeout_reg = 0;
    s->int_ena_reg = 0;
    s->int_raw_reg = 0;
    s->sda_hold_reg = 0;
    s->sda_sample_reg = 0;
    s->high_period_reg = 0;
    s->low_period_reg = 0;
    s->start_hold_reg = 0;
    s->rstart_setup_reg = 0;
    s->stop_hold_reg = 0;
    s->stop_setup_reg = 0;
    memset(s->cmd_reg, 0, sizeof(s->cmd_reg));

    fifo8_reset(&s->tx_fifo);
    fifo8_reset(&s->rx_fifo);
}

static const MemoryRegionOps esp32_i2c_ops = {
    .read = esp32_i2c_read,
    .write = esp32_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_i2c_init(Object * obj)
{
    Esp32I2CState *s = Esp32_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_i2c_ops, s, TYPE_ESP32_I2C, ESP32_I2C_MEM_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->bus = i2c_init_bus(DEVICE(s), "i2c");

    fifo8_create( &s->tx_fifo, ESP32_I2C_FIFO_LENGTH );
    fifo8_create( &s->rx_fifo, ESP32_I2C_FIFO_LENGTH );

    timer_init_ns( &s->event_timer, QEMU_CLOCK_VIRTUAL, esp32_i2c_event, s );
}

static void esp32_i2c_class_init(ObjectClass* klass, void* data)
{
    DeviceClass * dc = DEVICE_CLASS(klass);
    dc->reset = esp32_i2c_reset;
}

static const TypeInfo esp32_i2c_type_info = {
    .name = TYPE_ESP32_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32I2CState),
    .instance_init = esp32_i2c_init,
    .class_init = esp32_i2c_class_init,
};

static void esp32_i2c_register_types(void)
{
    type_register_static(&esp32_i2c_type_info);
}

type_init(esp32_i2c_register_types)
