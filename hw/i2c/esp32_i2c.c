#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "hw/i2c/esp32_i2c.h"
#include "hw/irq.h"

#include "../softmmu/simuliface.h"
#include "../xtensa/esp32-simul.h"


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
        //writeReg( (s->iomem.addr & 0x000FFFFF)+A_I2C_CMD, cmd ); // dfOne at write_CTR()
        s->bytesTx = 0;
        s->sr_reg |= 1<<4;                // I2C_BUS_BUSY
        //printf("Qemu: esp32_i2c CMD Start\n"); fflush( stdout );
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
        writeReg( (s->iomem.addr & 0x000FFFFF)+A_I2C_CMD, (cmd & ~0xFF) | data);
        s->int_raw_reg |= 1<<6;          // I2C_BYTE_TRANS
        time += (19*s->period_ns)/2;
    }break;

    case I2C_OPCODE_READ:
    {
        //writeReg( (s->iomem.addr & 0x000FFFFF)+A_I2C_CMD, cmd );
        /// size_t length = FIELD_EX32( cmd, I2C_CMD, BYTE_NUM );

        /// for( size_t nbytes=0; nbytes<length; ++nbytes)
        /// {
        ///     if( fifo8_num_free( &s->rx_fifo ) == 0 ) error_report("esp32_i2c: RX FIFO overflow");
        ///     else {
        ///         /// uint8_t data = i2c_recv( s->bus );
        ///         /// fifo8_push(&s->rx_fifo, data);
        ///     }
        /// }
    }break;

    case I2C_OPCODE_STOP:
    {
        //printf("Qemu: esp32_i2c CMD stop \n" ); fflush( stdout );
        writeReg( (s->iomem.addr & 0x000FFFFF)+A_I2C_CMD, cmd );
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

    uint8_t ackT = 0;
    uint8_t ackR = 0;

    //printf("Qemu: esp32_i2c_event %i %lu\n", s->lastOpcode, getQemu_ps() ); fflush( stdout );

    uint64_t status = readReg( (s->iomem.addr & 0x000FFFFF)+A_I2C_STATUS );
    /// TODO: get ACK from status

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

        bool ackERR = false;
        uint32_t cmd = s->cmd_reg[s->lastCMD];

        if( cmd & 1<<8 )                      // ACK_CHECK_EN
        {
            bool ackEXP = (cmd & 1<<9) == 0;  // ACK_EXP
            ackERR = !ackEXP;
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


    if( s->lastOpcode == I2C_OPCODE_WRITE && s->bytesTx )
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
    writeReg( (s->iomem.addr & 0x000FFFFF)+A_I2C_CTR, newCTR );

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
    writeReg( (s->iomem.addr & 0x000FFFFF)+A_I2C_LOW_PERIOD, period_ns );
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
