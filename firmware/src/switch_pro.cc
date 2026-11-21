// Independent, experimental implementation of Nintendo's USB Switch Pro
// protocol, based on the publicly documented USB command/report formats.
// Use with the HID Remapper's existing internal HORIPAD output mapping.
// Not a guarantee of compatibility with Switch 2 until hardware-tested.
#include "switch_pro.h"

#include <string.h>
#include "pico/time.h"

// The Pro HID report descriptor describes USB report IDs, *not* the internal
// remapper mapping layout. Our virtual HORIPAD layout is kept separately.
// The following wire descriptor follows the publicly documented 203-byte
// Nintendo Pro Controller HID report format.
const uint8_t switch_pro_report_descriptor[] = {
    0x05,0x01,0x15,0x00,0x09,0x04,0xA1,0x01,0x85,0x30,0x05,0x01,
    0x05,0x09,0x19,0x01,0x29,0x0A,0x15,0x00,0x25,0x01,0x75,0x01,
    0x95,0x0A,0x55,0x00,0x65,0x00,0x81,0x02,0x05,0x09,0x19,0x0B,
    0x29,0x0E,0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x04,0x81,0x02,
    0x75,0x01,0x95,0x02,0x81,0x03,0x0B,0x01,0x00,0x01,0x00,0xA1,
    0x00,0x0B,0x30,0x00,0x01,0x00,0x0B,0x31,0x00,0x01,0x00,0x0B,
    0x32,0x00,0x01,0x00,0x0B,0x35,0x00,0x01,0x00,0x15,0x00,0x27,
    0xFF,0xFF,0x00,0x00,0x75,0x10,0x95,0x04,0x81,0x02,0xC0,0x0B,
    0x39,0x00,0x01,0x00,0x15,0x00,0x25,0x07,0x35,0x00,0x46,0x3B,
    0x01,0x65,0x14,0x75,0x04,0x95,0x01,0x81,0x42,0x05,0x09,0x19,
    0x0F,0x29,0x12,0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x04,0x81,
    0x02,0x75,0x08,0x95,0x34,0x81,0x03,0x06,0x00,0xFF,0x85,0x21,
    0x09,0x01,0x75,0x08,0x95,0x3F,0x81,0x03,0x85,0x81,0x09,0x02,
    0x75,0x08,0x95,0x3F,0x81,0x03,0x85,0x01,0x09,0x03,0x75,0x08,
    0x95,0x3F,0x91,0x83,0x85,0x10,0x09,0x04,0x75,0x08,0x95,0x3F,
    0x91,0x83,0x85,0x80,0x09,0x05,0x75,0x08,0x95,0x3F,0x91,0x83,
    0x85,0x82,0x09,0x06,0x75,0x08,0x95,0x3F,0x91,0x83,0xC0
};
static_assert(sizeof(switch_pro_report_descriptor) == SWITCH_PRO_REPORT_DESCRIPTOR_SIZE,
              "Unexpected Nintendo Pro Controller HID report descriptor size");

namespace {
constexpr uint8_t REPORT_LEN = 63;
constexpr uint8_t QUEUE_CAP = 8;
constexpr uint32_t PERIOD_MS = 8;
constexpr uint16_t STEAM_VID = 0x28DE;
constexpr uint8_t MAC[6] = {0x7C,0xBB,0x8A,0x43,0x21,0x10};

struct Motion {
    int16_t ax, ay, az, gx, gy, gz;
};
struct Reply {
    uint8_t id;
    uint8_t data[REPORT_LEN];
};

uint8_t hori[8] = {0,0,0x0F,0x80,0x80,0x80,0x80,0};
Motion motion[3] = {};
Reply replies[QUEUE_CAP] = {};
uint8_t head = 0, tail = 0;
uint8_t timer_value = 0;
uint8_t mode = 0;
bool usb_running = false;
bool user_cal_initialized = false;
uint8_t user_cal[256] = {};
uint32_t last_stream_ms = 0;

uint32_t ms_now() {
    return to_ms_since_boot(get_absolute_time());
}
int16_t read_i16(const uint8_t* bytes) {
    return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}
void write_i16(uint8_t *p, int16_t value) {
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)((uint16_t)value >> 8);
}
void reply_enqueue(uint8_t id, const uint8_t *data, uint8_t len) {
    uint8_t next = (uint8_t)((tail + 1) % QUEUE_CAP);
    if (next == head) return;
    Reply &slot = replies[tail];
    slot.id = id;
    memset(slot.data, 0, REPORT_LEN);
    if (len > REPORT_LEN) len = REPORT_LEN;
    if (data != nullptr && len) memcpy(slot.data, data, len);
    tail = next;
}
uint16_t stick_from_byte(uint8_t v) {
    int32_t scaled = 2048 + ((int32_t)v - 128) * 16;
    if (scaled < 0) return 0;
    if (scaled > 4095) return 4095;
    return (uint16_t)scaled;
}
void pack_stick(uint8_t* out, uint8_t x, uint8_t y) {
    uint16_t a = stick_from_byte(x), b = stick_from_byte(y);
    out[0] = (uint8_t)a;
    out[1] = (uint8_t)((a >> 8) | (b << 4));
    out[2] = (uint8_t)(b >> 4);
}
void pack_12x6(uint8_t *out, const uint16_t *v) {
    for (uint8_t i=0; i<3; ++i) {
        uint16_t a=v[2*i], b=v[2*i+1];
        out[i*3]   = (uint8_t)a;
        out[i*3+1] = (uint8_t)((a>>8)|(b<<4));
        out[i*3+2] = (uint8_t)(b>>4);
    }
}
void spi_read(uint32_t address, uint8_t count, uint8_t *dst) {
    // Factory IMU: 1 g = 4096 units, gyro sensitivity = 0x343B.
    static constexpr uint8_t IMU[24] = {
        0,0, 0,0, 0,0, 0,0x40, 0,0x40, 0,0x40,
        0,0, 0,0, 0,0, 0x3B,0x34, 0x3B,0x34, 0x3B,0x34
    };
    static constexpr uint8_t DEVICE_PARAMS1[24] = {
        0x50,0xFD,0x00,0x00,0xC6,0x0F,0x0F,0x30,
        0x61,0x96,0x30,0xF3,0xD4,0x14,0x54,0x41,
        0x15,0x54,0xC7,0x79,0x9C,0x33,0x36,0x63
    };
    static constexpr uint8_t DEVICE_PARAMS2[18] = {
        0x0F,0x30,0x61,0x96,0x30,0xF3,0xD4,0x14,0x54,
        0x41,0x15,0x54,0xC7,0x79,0x9C,0x33,0x36,0x63
    };
    static constexpr uint8_t COLORS[13] = {
        0x32,0x32,0x32,0xE6,0xE6,0xE6,0x32,
        0x32,0x32,0x32,0x32,0x32,0xFF
    };
    const uint16_t left[6] = {1800,1800,2048,2048,1800,1800};
    const uint16_t right[6] = {2048,2048,1800,1800,1800,1800};
    uint8_t sticks[18];
    pack_12x6(sticks, left);
    pack_12x6(sticks+9, right);
    for (uint8_t i=0; i<count; ++i) {
        uint32_t a=address+i;
        uint8_t v=0xFF;
        if (a>=0x6020 && a<0x6038) v=IMU[a-0x6020];
        else if (a>=0x603D && a<0x604F) v=sticks[a-0x603D];
        else if (a>=0x6050 && a<0x605D) v=COLORS[a-0x6050];
        else if (a>=0x6080 && a<0x6098) v=DEVICE_PARAMS1[a-0x6080];
        else if (a>=0x6098 && a<0x60AA) v=DEVICE_PARAMS2[a-0x6098];
        else if (a>=0x8000 && a<0x8100) v=user_cal[a-0x8000];
        dst[i]=v;
    }
}
uint32_t mapped_buttons() {
    // HORIPAD internal bits: Y,B,A,X,L,R,ZL,ZR,-,+,L3,R3,Home,Capture.
    // Pro 0x30: Right-buttons byte, shared byte, left-buttons byte.
    uint16_t b=(uint16_t)hori[0]|((uint16_t)hori[1]<<8);
    uint32_t o=0;
    if (b & (1u<<0)) o|=1u<<0;   // Y
    if (b & (1u<<3)) o|=1u<<1;   // X
    if (b & (1u<<1)) o|=1u<<2;   // B
    if (b & (1u<<2)) o|=1u<<3;   // A
    if (b & (1u<<5)) o|=1u<<6;   // R
    if (b & (1u<<7)) o|=1u<<7;   // ZR
    if (b & (1u<<8)) o|=1u<<8;   // Minus
    if (b & (1u<<9)) o|=1u<<9;   // Plus
    if (b & (1u<<11))o|=1u<<10;  // RStick
    if (b & (1u<<10))o|=1u<<11;  // LStick
    if (b & (1u<<12))o|=1u<<12;  // Home
    if (b & (1u<<13))o|=1u<<13;  // Capture
    if (b & (1u<<4)) o|=1u<<22;  // L
    if (b & (1u<<6)) o|=1u<<23;  // ZL
    const uint8_t hat=hori[2]&0x0F;
    if (hat<=7) {
        if (hat==0 || hat==1 || hat==7) o|=1u<<17; // Up
        if (hat==1 || hat==2 || hat==3) o|=1u<<18; // Right
        if (hat==3 || hat==4 || hat==5) o|=1u<<16; // Down
        if (hat==5 || hat==6 || hat==7) o|=1u<<19; // Left
    }
    return o;
}
void make_prefix(uint8_t out[REPORT_LEN]) {
    out[0]=timer_value++;
    out[1]=0x81; // full battery, USB powered
    uint32_t buttons=mapped_buttons();
    out[2]=(uint8_t)buttons;
    out[3]=(uint8_t)(buttons>>8);
    out[4]=(uint8_t)(buttons>>16);
    pack_stick(out+5,hori[3],hori[4]);
    pack_stick(out+8,hori[5],hori[6]);
    out[11]=0x09;
}
void make_full(uint8_t out[REPORT_LEN]) {
    memset(out, 0, REPORT_LEN);
    make_prefix(out);
    for (uint8_t k=0; k<3; ++k) {
        const Motion &m=motion[k];
        uint8_t *p=out+12+k*12;
        // Sensor orientation: controller axis -> Pro axis.
        // Adapt signs if motion test on hardware finds reversed axes.
        write_i16(p+0, (int16_t)((int32_t)m.ay/4));
        write_i16(p+2, (int16_t)(-(int32_t)m.ax/4));
        write_i16(p+4, (int16_t)((int32_t)m.az/4));
        write_i16(p+6, m.gy);
        write_i16(p+8, (int16_t)(-(int32_t)m.gx));
        write_i16(p+10,m.gz);
    }
}
void make_subcommand_reply(uint8_t sub, const uint8_t *args, uint16_t args_len) {
    uint8_t out[REPORT_LEN] = {};
    make_prefix(out);
    out[13]=sub;
    out[12]=0x80;
    switch (sub) {
        case 0x01: { // manual pairing stage
            uint8_t stage=args_len?args[0]:3;
            out[12]=0x81;
            out[14]=stage;
            if (stage==1) {
                memcpy(out+15,MAC,6);
                out[22]=0x25; out[23]=0x08;
                static const char name[]="Pro Controller";
                memcpy(out+24,name,sizeof(name)-1);
                out[43]=0x68;
            } else if (stage==2) {
                static const uint8_t pairing_bytes[16] = {
                    0xE5,0xC8,0xE4,0x92,0x05,0xFF,0xC9,0x8A,
                    0x7D,0xEA,0x15,0xF6,0x19,0xBA,0x82,0x13
                };
                memcpy(out+15,pairing_bytes,16);
            }
            break;
        }
        case 0x02: // device info
            out[12]=0x82;
            out[14]=0x03; out[15]=0x48; // firmware 3.72
            out[16]=0x03; // Pro Controller
            out[17]=0x02;
            memcpy(out+18,MAC,6);
            out[24]=0x01; out[25]=0x01;
            break;
        case 0x03: // input mode
            if (args_len && args[0]==0x30) mode=0x30;
            break;
        case 0x04: // buttons elapsed time
            out[12]=0x83;
            out[15]=0xCC; out[17]=0xEE; out[19]=0xFF;
            break;
        case 0x10: { // read SPI flash
            if (args_len<5) break;
            uint32_t address=(uint32_t)args[0]|((uint32_t)args[1]<<8)|
                             ((uint32_t)args[2]<<16)|((uint32_t)args[3]<<24);
            uint8_t count=args[4];
            if (count>29) count=29;
            out[12]=0x90;
            memcpy(out+14,args,4);
            out[18]=count;
            spi_read(address,count,out+19);
            break;
        }
        case 0x11: { // write user SPI calibration; RAM until power-cycle
            if (args_len>=5) {
                uint32_t address=(uint32_t)args[0]|((uint32_t)args[1]<<8)|
                                 ((uint32_t)args[2]<<16)|((uint32_t)args[3]<<24);
                uint8_t count=args[4];
                for (uint16_t i=0;i<count && i+5<args_len;++i) {
                    uint32_t target=address+i;
                    if (target>=0x8000 && target<0x8100)
                        user_cal[target-0x8000]=args[5+i];
                }
            }
            break;
        }
        case 0x21: out[12]=0xA0; break; // NFC/IR config
        default: break; // 0x08,0x30,0x38,0x40,0x41,0x48 etc.
    }
    reply_enqueue(0x21,out,REPORT_LEN);
}
} // namespace

void switch_pro_init() {
    static constexpr uint8_t neutral[8] = {0,0,15,128,128,128,128,0};
    memcpy(hori, neutral, sizeof neutral);
    memset(motion, 0, sizeof(motion));
    if (!user_cal_initialized) {
        memset(user_cal, 0xFF, sizeof(user_cal));
        user_cal_initialized=true;
    }
    switch_pro_reset_session();
}

void switch_pro_reset_session() {
    head=tail=0;
    mode=0;
    usb_running=false;
    timer_value=0;
    last_stream_ms=0;
}

void switch_pro_set_horipad(const uint8_t *data, uint8_t len) {
    if (data && len>=8) memcpy(hori,data,8);
}

void switch_pro_capture_steam(uint8_t dev_addr, uint8_t const* report, uint16_t len) {
    if (!report || len<46 || (report[0]!=0x42 && report[0]!=0x45)) return;
    uint16_t vid=0, pid=0;
    tuh_vid_pid_get(dev_addr,&vid,&pid);
    if (vid!=STEAM_VID || (pid!=0x1302 && pid!=0x1303 && pid!=0x1304)) return;
    // quirks.cc: report ID 66/69, data starts after the report ID.
    // Bit positions: acc 264,280,296; gyro 312,328,344.
    // 264/8 = 33; the leading report ID adds one byte.
    Motion newest = {
        read_i16(report+34), read_i16(report+36), read_i16(report+38),
        read_i16(report+40), read_i16(report+42), read_i16(report+44)
    };
    motion[0]=motion[1];
    motion[1]=motion[2];
    motion[2]=newest;
}

void switch_pro_handle_output(uint8_t report_id, hid_report_type_t type,
                              const uint8_t *buffer, uint16_t len) {
    if (type != HID_REPORT_TYPE_OUTPUT || !buffer || len == 0) return;
    // TinyUSB may hand us OUT endpoint data with the report ID in buffer[0].
    uint8_t id=report_id;
    if (id==0) { id=buffer[0]; ++buffer; --len; }
    if (id==0x80 && len) {
        switch (buffer[0]) {
            case 0x01: {
                uint8_t answer[9]={0x01,0x00,0x03};
                memcpy(answer+3,MAC,6);
                reply_enqueue(0x81,answer,sizeof(answer));
                break;
            }
            case 0x02: { const uint8_t answer=0x02; reply_enqueue(0x81,&answer,1); break; }
            case 0x03: { const uint8_t answer=0x03; reply_enqueue(0x81,&answer,1); break; }
            case 0x04: usb_running=true; break;
            case 0x06: switch_pro_reset_session(); break;
            default: break;
        }
    } else if (id==0x01 && len>=10) {
        // [timer][8 bytes rumble][subcommand][arguments...]
        make_subcommand_reply(buffer[9],buffer+10,len-10);
    } else if (id==0x10) {
        // Rumble only; intentionally not implemented yet.
    }
}

uint16_t switch_pro_get_report(uint8_t id, hid_report_type_t type,
                               uint8_t *buffer, uint16_t max_len) {
    if (type != HID_REPORT_TYPE_INPUT || !buffer) return 0;
    if (id==0x30 && max_len>=REPORT_LEN) {
        make_full(buffer);
        return REPORT_LEN;
    }
    return 0;
}

void switch_pro_task() {
    if (!tud_hid_n_ready(0)) return;
    if (head!=tail) {
        const Reply &r=replies[head];
        if (tud_hid_n_report(0,r.id,r.data,REPORT_LEN))
            head=(uint8_t)((head+1)%QUEUE_CAP);
        return;
    }
    if (!usb_running && mode!=0x30) return;
    uint32_t now=ms_now();
    if ((uint32_t)(now-last_stream_ms)<PERIOD_MS) return;
    last_stream_ms=now;
    uint8_t out[REPORT_LEN];
    make_full(out);
    tud_hid_n_report(0,0x30,out,REPORT_LEN);
}
