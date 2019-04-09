#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 149 "/usr/bin/../lib/gcc/msp430/4.5.3/include/stddef.h" 3
typedef long int ptrdiff_t;
#line 211
typedef unsigned int size_t;
#line 323
typedef int wchar_t;
# 8 "/usr/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
#line 8
  int dummy;
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
#line 13
  int dummy;
}  ;
#line 14
struct __nesc_attr_one_nok {
#line 14
  int dummy;
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
#line 17
  int dummy;
}  ;
# 38 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/stdint.h" 3
typedef signed char int8_t;
typedef int int16_t;
typedef long int int32_t;
__extension__ 
#line 41
typedef long long int int64_t;

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long int uint32_t;
__extension__ 
#line 46
typedef unsigned long long int uint64_t;





typedef signed char int_least8_t;
typedef int int_least16_t;
typedef long int int_least32_t;
__extension__ 
#line 55
typedef long long int int_least64_t;


typedef unsigned char uint_least8_t;
typedef unsigned int uint_least16_t;
typedef unsigned long int uint_least32_t;
__extension__ 
#line 61
typedef unsigned long long int uint_least64_t;





typedef signed char int_fast8_t;
typedef int int_fast16_t;
typedef long int int_fast32_t;
__extension__ 
#line 70
typedef long long int int_fast64_t;


typedef unsigned char uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned long int uint_fast32_t;
__extension__ 
#line 76
typedef unsigned long long int uint_fast64_t;









typedef int16_t intptr_t;
typedef uint16_t uintptr_t;





__extension__ 
#line 93
typedef long long int intmax_t;
__extension__ 
#line 94
typedef unsigned long long int uintmax_t;
# 281 "/usr/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_ntoh_uint8(const void * source)  ;




static __inline uint8_t __nesc_hton_uint8(void * target, uint8_t value)  ;





static __inline uint8_t __nesc_ntoh_leuint8(const void * source)  ;




static __inline uint8_t __nesc_hton_leuint8(void * target, uint8_t value)  ;
#line 322
static __inline uint16_t __nesc_ntoh_leuint16(const void * source)  ;




static __inline uint16_t __nesc_hton_leuint16(void * target, uint16_t value)  ;
#line 340
static __inline uint32_t __nesc_ntoh_uint32(const void * source)  ;






static __inline uint32_t __nesc_hton_uint32(void * target, uint32_t value)  ;
#line 372
static __inline int32_t __nesc_hton_int32(void * target, int32_t value)  ;
#line 431
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 48 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/sys/types.h" 3
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 40 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/string.h" 3
extern void *memset(void *arg_0x2b3959b6a980, int arg_0x2b3959b6abe8, size_t arg_0x2b3959b69020);
#line 61
extern void *memset(void *arg_0x2b3959b83b10, int arg_0x2b3959b83d78, size_t arg_0x2b3959b82060);
# 59 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/stdlib.h" 3
#line 55
typedef struct __nesc_unnamed4242 {

  int quot;
  int rem;
} div_t;







#line 63
typedef struct __nesc_unnamed4243 {

  long quot;
  long rem;
} ldiv_t;
# 122 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 19 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/sys/reent.h" 3
typedef unsigned long __ULong;
#line 31
struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0x2b3959bc4290);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x2b3959bc9300);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/math.h" 3
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 220
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 273
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 25 "/opt/tinyos-2.1.2/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 26
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;
uint16_t TOS_NODE_ID = 1;






struct __nesc_attr_atmostonce {
};
#line 37
struct __nesc_attr_atleastonce {
};
#line 38
struct __nesc_attr_exactlyonce {
};
# 51 "/opt/tinyos-2.1.2/tos/types/TinyError.h"
enum __nesc_unnamed4248 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 30 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/msp430.h" 3
#line 24
typedef enum msp430_cpu_e {

  MSP430_CPU_MSP430 = 0x0000, 
  MSP430_CPU_MSP430X = 0x0002, 
  MSP430_CPU_MSP430XV2 = 0x0003, 
  MSP430_CPU = 0x0003
} msp430_cpu_e;
#line 46
#line 34
typedef enum msp430_mpy_e {

  MSP430_MPY_NONE = 0x0000, 
  MSP430_MPY_TYPE_16 = 0x0010, 
  MSP430_MPY_TYPE_32 = 0x0020, 
  MSP430_MPY_TYPE_ANY = 0x0030, 
  MSP430_MPY_HAS_SE = 0x0001, 
  MSP430_MPY_HAS_DW = 0x0002, 
  MSP430_MPY_16 = MSP430_MPY_TYPE_16, 
  MSP430_MPY_16SE = MSP430_MPY_16 | MSP430_MPY_HAS_SE, 
  MSP430_MPY_32 = MSP430_MPY_TYPE_32 | MSP430_MPY_HAS_SE, 
  MSP430_MPY_32DW = MSP430_MPY_32 | MSP430_MPY_HAS_DW
} msp430_mpy_e;
# 43 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/in430.h" 3
void __nop(void );



void __dint(void );



void __eint(void );


unsigned int __read_status_register(void );
# 162 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/msp430f1611.h" 3
extern volatile unsigned char ME1 __asm ("__""ME1");
#line 181
extern volatile unsigned char ME2 __asm ("__""ME2");
#line 193
extern volatile unsigned int WDTCTL __asm ("__""WDTCTL");
#line 265
extern volatile unsigned char P1OUT __asm ("__""P1OUT");

extern volatile unsigned char P1DIR __asm ("__""P1DIR");

extern volatile unsigned char P1IFG __asm ("__""P1IFG");



extern volatile unsigned char P1IE __asm ("__""P1IE");

extern volatile unsigned char P1SEL __asm ("__""P1SEL");




extern volatile unsigned char P2OUT __asm ("__""P2OUT");

extern volatile unsigned char P2DIR __asm ("__""P2DIR");

extern volatile unsigned char P2IFG __asm ("__""P2IFG");



extern volatile unsigned char P2IE __asm ("__""P2IE");

extern volatile unsigned char P2SEL __asm ("__""P2SEL");










extern volatile unsigned char P3OUT __asm ("__""P3OUT");

extern volatile unsigned char P3DIR __asm ("__""P3DIR");

extern volatile unsigned char P3SEL __asm ("__""P3SEL");




extern volatile unsigned char P4OUT __asm ("__""P4OUT");

extern volatile unsigned char P4DIR __asm ("__""P4DIR");

extern volatile unsigned char P4SEL __asm ("__""P4SEL");










extern volatile unsigned char P5OUT __asm ("__""P5OUT");

extern volatile unsigned char P5DIR __asm ("__""P5DIR");

extern volatile unsigned char P5SEL __asm ("__""P5SEL");




extern volatile unsigned char P6OUT __asm ("__""P6OUT");

extern volatile unsigned char P6DIR __asm ("__""P6DIR");

extern volatile unsigned char P6SEL __asm ("__""P6SEL");
#line 380
extern volatile unsigned char U0CTL __asm ("__""U0CTL");

extern volatile unsigned char U0TCTL __asm ("__""U0TCTL");



extern volatile unsigned char U0MCTL __asm ("__""U0MCTL");

extern volatile unsigned char U0BR0 __asm ("__""U0BR0");

extern volatile unsigned char U0BR1 __asm ("__""U0BR1");

extern const volatile unsigned char U0RXBUF __asm ("__""U0RXBUF");
#line 439
extern volatile unsigned char U1TCTL __asm ("__""U1TCTL");
#line 593
extern volatile unsigned int TACTL __asm ("__""TACTL");

extern volatile unsigned int TACCTL0 __asm ("__""TACCTL0");





extern volatile unsigned int TAR __asm ("__""TAR");

extern volatile unsigned int TACCR0 __asm ("__""TACCR0");
#line 695
extern volatile unsigned int TBCTL __asm ("__""TBCTL");

extern volatile unsigned int TBCCTL0 __asm ("__""TBCCTL0");

extern volatile unsigned int TBCCTL1 __asm ("__""TBCCTL1");

extern volatile unsigned int TBCCTL2 __asm ("__""TBCCTL2");









extern volatile unsigned int TBR __asm ("__""TBR");
#line 790
extern volatile unsigned char DCOCTL __asm ("__""DCOCTL");

extern volatile unsigned char BCSCTL1 __asm ("__""BCSCTL1");

extern volatile unsigned char BCSCTL2 __asm ("__""BCSCTL2");
#line 962
extern volatile unsigned int ADC12CTL0 __asm ("__""ADC12CTL0");

extern volatile unsigned int ADC12CTL1 __asm ("__""ADC12CTL1");
# 343 "/opt/tinyos-2.1.2/tos/chips/msp430/msp430hardware.h"
static volatile uint8_t U0CTLnr __asm ("0x0070");
static volatile uint8_t I2CTCTLnr __asm ("0x0071");
static volatile uint8_t I2CDCTLnr __asm ("0x0072");
#line 378
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)  ;


enum __nesc_unnamed4249 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void )  ;





static inline void __nesc_enable_interrupt(void )  ;




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )   ;







void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)   ;
#line 433
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_float;typedef float __nesc_nxbase_nx_float  ;
#line 448
enum __nesc_unnamed4250 {
  MSP430_PORT_RESISTOR_INVALID, 
  MSP430_PORT_RESISTOR_OFF, 
  MSP430_PORT_RESISTOR_PULLDOWN, 
  MSP430_PORT_RESISTOR_PULLUP
};
# 8 "/opt/tinyos-2.1.2/tos/platforms/telosb/hardware.h"
enum __nesc_unnamed4251 {
  TOS_SLEEP_NONE = MSP430_POWER_ACTIVE
};
#line 36
static inline void TOSH_SET_SIMO0_PIN()  ;
#line 36
static inline void TOSH_CLR_SIMO0_PIN()  ;
#line 36
static inline void TOSH_MAKE_SIMO0_OUTPUT()  ;
static inline void TOSH_SET_UCLK0_PIN()  ;
#line 37
static inline void TOSH_CLR_UCLK0_PIN()  ;
#line 37
static inline void TOSH_MAKE_UCLK0_OUTPUT()  ;
#line 79
enum __nesc_unnamed4252 {

  TOSH_HUMIDITY_ADDR = 5, 
  TOSH_HUMIDTEMP_ADDR = 3, 
  TOSH_HUMIDITY_RESET = 0x1E
};



static inline void TOSH_SET_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_CLR_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_MAKE_FLASH_CS_OUTPUT()  ;
static inline void TOSH_SET_FLASH_HOLD_PIN()  ;
#line 89
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT()  ;
# 41 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4253 {
#line 41
  int notUsed;
} 
#line 41
TSecond;
typedef struct __nesc_unnamed4254 {
#line 42
  int notUsed;
} 
#line 42
TMilli;
typedef struct __nesc_unnamed4255 {
#line 43
  int notUsed;
} 
#line 43
T32khz;
typedef struct __nesc_unnamed4256 {
#line 44
  int notUsed;
} 
#line 44
TMicro;
# 6 "BlinkToRadio.h"
enum __nesc_unnamed4257 {
  AM_BLINKTORADIOMSG = 6, 
  TIMER_PERIOD_MILLI = 100
};










#line 13
typedef nx_struct BlinkToRadioMsg {
  nx_uint8_t nodeid;
  nx_uint8_t counter;
  nx_uint32_t sendTS;
  nx_uint32_t receiveTS;
  nx_uint32_t sendTS_of_member;
  nx_uint32_t receiveTS_of_head;
} __attribute__((packed)) BlinkToRadioMsg;
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4258 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 75
#line 62
typedef struct __nesc_unnamed4259 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} msp430_compare_control_t;
#line 87
#line 77
typedef struct __nesc_unnamed4260 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
#line 102
#line 89
typedef struct __nesc_unnamed4261 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} msp430_timer_b_control_t;
# 43 "/opt/tinyos-2.1.2/tos/types/Leds.h"
enum __nesc_unnamed4262 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 39 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420.h"
typedef uint8_t cc2420_status_t;
#line 93
#line 87
typedef nx_struct security_header_t {
  unsigned char __nesc_filler0[1];


  nx_uint32_t frameCounter;
  nx_uint8_t keyID[1];
} __attribute__((packed)) security_header_t;
#line 113
#line 95
typedef nx_struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;







  nxle_uint8_t network;


  nxle_uint8_t type;
} __attribute__((packed)) cc2420_header_t;





#line 118
typedef nx_struct cc2420_footer_t {
} __attribute__((packed)) cc2420_footer_t;
#line 143
#line 128
typedef nx_struct cc2420_metadata_t {
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_uint8_t tx_power;
  nx_bool crc;
  nx_bool ack;
  nx_bool timesync;
  nx_uint32_t timestamp;
  nx_uint16_t rxInterval;
} __attribute__((packed)) 





cc2420_metadata_t;





#line 146
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t data[];
} __attribute__((packed)) cc2420_packet_t;
#line 179
enum __nesc_unnamed4263 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 28 + MAC_FOOTER_SIZE, 

  CC2420_SIZE = MAC_HEADER_SIZE + MAC_FOOTER_SIZE
};

enum cc2420_enums {
  CC2420_TIME_ACK_TURNAROUND = 7, 
  CC2420_TIME_VREN = 20, 
  CC2420_TIME_SYMBOL = 2, 
  CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, 
  CC2420_MIN_BACKOFF = 20 / CC2420_TIME_SYMBOL, 
  CC2420_ACK_WAIT_DELAY = 256
};

enum cc2420_status_enums {
  CC2420_STATUS_RSSI_VALID = 1 << 1, 
  CC2420_STATUS_LOCK = 1 << 2, 
  CC2420_STATUS_TX_ACTIVE = 1 << 3, 
  CC2420_STATUS_ENC_BUSY = 1 << 4, 
  CC2420_STATUS_TX_UNDERFLOW = 1 << 5, 
  CC2420_STATUS_XOSC16M_STABLE = 1 << 6
};

enum cc2420_config_reg_enums {
  CC2420_SNOP = 0x00, 
  CC2420_SXOSCON = 0x01, 
  CC2420_STXCAL = 0x02, 
  CC2420_SRXON = 0x03, 
  CC2420_STXON = 0x04, 
  CC2420_STXONCCA = 0x05, 
  CC2420_SRFOFF = 0x06, 
  CC2420_SXOSCOFF = 0x07, 
  CC2420_SFLUSHRX = 0x08, 
  CC2420_SFLUSHTX = 0x09, 
  CC2420_SACK = 0x0a, 
  CC2420_SACKPEND = 0x0b, 
  CC2420_SRXDEC = 0x0c, 
  CC2420_STXENC = 0x0d, 
  CC2420_SAES = 0x0e, 
  CC2420_MAIN = 0x10, 
  CC2420_MDMCTRL0 = 0x11, 
  CC2420_MDMCTRL1 = 0x12, 
  CC2420_RSSI = 0x13, 
  CC2420_SYNCWORD = 0x14, 
  CC2420_TXCTRL = 0x15, 
  CC2420_RXCTRL0 = 0x16, 
  CC2420_RXCTRL1 = 0x17, 
  CC2420_FSCTRL = 0x18, 
  CC2420_SECCTRL0 = 0x19, 
  CC2420_SECCTRL1 = 0x1a, 
  CC2420_BATTMON = 0x1b, 
  CC2420_IOCFG0 = 0x1c, 
  CC2420_IOCFG1 = 0x1d, 
  CC2420_MANFIDL = 0x1e, 
  CC2420_MANFIDH = 0x1f, 
  CC2420_FSMTC = 0x20, 
  CC2420_MANAND = 0x21, 
  CC2420_MANOR = 0x22, 
  CC2420_AGCCTRL = 0x23, 
  CC2420_AGCTST0 = 0x24, 
  CC2420_AGCTST1 = 0x25, 
  CC2420_AGCTST2 = 0x26, 
  CC2420_FSTST0 = 0x27, 
  CC2420_FSTST1 = 0x28, 
  CC2420_FSTST2 = 0x29, 
  CC2420_FSTST3 = 0x2a, 
  CC2420_RXBPFTST = 0x2b, 
  CC2420_FMSTATE = 0x2c, 
  CC2420_ADCTST = 0x2d, 
  CC2420_DACTST = 0x2e, 
  CC2420_TOPTST = 0x2f, 
  CC2420_TXFIFO = 0x3e, 
  CC2420_RXFIFO = 0x3f
};

enum cc2420_ram_addr_enums {
  CC2420_RAM_TXFIFO = 0x000, 
  CC2420_RAM_RXFIFO = 0x080, 
  CC2420_RAM_KEY0 = 0x100, 
  CC2420_RAM_RXNONCE = 0x110, 
  CC2420_RAM_SABUF = 0x120, 
  CC2420_RAM_KEY1 = 0x130, 
  CC2420_RAM_TXNONCE = 0x140, 
  CC2420_RAM_CBCSTATE = 0x150, 
  CC2420_RAM_IEEEADR = 0x160, 
  CC2420_RAM_PANID = 0x168, 
  CC2420_RAM_SHORTADR = 0x16a
};

enum cc2420_nonce_enums {
  CC2420_NONCE_BLOCK_COUNTER = 0, 
  CC2420_NONCE_KEY_SEQ_COUNTER = 2, 
  CC2420_NONCE_FRAME_COUNTER = 3, 
  CC2420_NONCE_SOURCE_ADDRESS = 7, 
  CC2420_NONCE_FLAGS = 15
};

enum cc2420_main_enums {
  CC2420_MAIN_RESETn = 15, 
  CC2420_MAIN_ENC_RESETn = 14, 
  CC2420_MAIN_DEMOD_RESETn = 13, 
  CC2420_MAIN_MOD_RESETn = 12, 
  CC2420_MAIN_FS_RESETn = 11, 
  CC2420_MAIN_XOSC16M_BYPASS = 0
};

enum cc2420_mdmctrl0_enums {
  CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, 
  CC2420_MDMCTRL0_PAN_COORDINATOR = 12, 
  CC2420_MDMCTRL0_ADR_DECODE = 11, 
  CC2420_MDMCTRL0_CCA_HYST = 8, 
  CC2420_MDMCTRL0_CCA_MOD = 6, 
  CC2420_MDMCTRL0_AUTOCRC = 5, 
  CC2420_MDMCTRL0_AUTOACK = 4, 
  CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0
};

enum cc2420_mdmctrl1_enums {
  CC2420_MDMCTRL1_CORR_THR = 6, 
  CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, 
  CC2420_MDMCTRL1_MODULATION_MODE = 4, 
  CC2420_MDMCTRL1_TX_MODE = 2, 
  CC2420_MDMCTRL1_RX_MODE = 0
};

enum cc2420_rssi_enums {
  CC2420_RSSI_CCA_THR = 8, 
  CC2420_RSSI_RSSI_VAL = 0
};

enum cc2420_syncword_enums {
  CC2420_SYNCWORD_SYNCWORD = 0
};

enum cc2420_txctrl_enums {
  CC2420_TXCTRL_TXMIXBUF_CUR = 14, 
  CC2420_TXCTRL_TX_TURNAROUND = 13, 
  CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, 
  CC2420_TXCTRL_TXMIX_CURRENT = 9, 
  CC2420_TXCTRL_PA_CURRENT = 6, 
  CC2420_TXCTRL_RESERVED = 5, 
  CC2420_TXCTRL_PA_LEVEL = 0
};

enum cc2420_rxctrl0_enums {
  CC2420_RXCTRL0_RXMIXBUF_CUR = 12, 
  CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, 
  CC2420_RXCTRL0_MED_LNA_GAIN = 8, 
  CC2420_RXCTRL0_LOW_LNA_GAIN = 6, 
  CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, 
  CC2420_RXCTRL0_MED_LNA_CURRENT = 2, 
  CC2420_RXCTRL0_LOW_LNA_CURRENT = 0
};

enum cc2420_rxctrl1_enums {
  CC2420_RXCTRL1_RXBPF_LOCUR = 13, 
  CC2420_RXCTRL1_RXBPF_MIDCUR = 12, 
  CC2420_RXCTRL1_LOW_LOWGAIN = 11, 
  CC2420_RXCTRL1_MED_LOWGAIN = 10, 
  CC2420_RXCTRL1_HIGH_HGM = 9, 
  CC2420_RXCTRL1_MED_HGM = 8, 
  CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, 
  CC2420_RXCTRL1_RXMIX_TAIL = 4, 
  CC2420_RXCTRL1_RXMIX_VCM = 2, 
  CC2420_RXCTRL1_RXMIX_CURRENT = 0
};

enum cc2420_rsctrl_enums {
  CC2420_FSCTRL_LOCK_THR = 14, 
  CC2420_FSCTRL_CAL_DONE = 13, 
  CC2420_FSCTRL_CAL_RUNNING = 12, 
  CC2420_FSCTRL_LOCK_LENGTH = 11, 
  CC2420_FSCTRL_LOCK_STATUS = 10, 
  CC2420_FSCTRL_FREQ = 0
};

enum cc2420_secctrl0_enums {
  CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, 
  CC2420_SECCTRL0_SEC_CBC_HEAD = 8, 
  CC2420_SECCTRL0_SEC_SAKEYSEL = 7, 
  CC2420_SECCTRL0_SEC_TXKEYSEL = 6, 
  CC2420_SECCTRL0_SEC_RXKEYSEL = 5, 
  CC2420_SECCTRL0_SEC_M = 2, 
  CC2420_SECCTRL0_SEC_MODE = 0
};

enum cc2420_secctrl1_enums {
  CC2420_SECCTRL1_SEC_TXL = 8, 
  CC2420_SECCTRL1_SEC_RXL = 0
};

enum cc2420_battmon_enums {
  CC2420_BATTMON_BATT_OK = 6, 
  CC2420_BATTMON_BATTMON_EN = 5, 
  CC2420_BATTMON_BATTMON_VOLTAGE = 0
};

enum cc2420_iocfg0_enums {
  CC2420_IOCFG0_BCN_ACCEPT = 11, 
  CC2420_IOCFG0_FIFO_POLARITY = 10, 
  CC2420_IOCFG0_FIFOP_POLARITY = 9, 
  CC2420_IOCFG0_SFD_POLARITY = 8, 
  CC2420_IOCFG0_CCA_POLARITY = 7, 
  CC2420_IOCFG0_FIFOP_THR = 0
};

enum cc2420_iocfg1_enums {
  CC2420_IOCFG1_HSSD_SRC = 10, 
  CC2420_IOCFG1_SFDMUX = 5, 
  CC2420_IOCFG1_CCAMUX = 0
};

enum cc2420_manfidl_enums {
  CC2420_MANFIDL_PARTNUM = 12, 
  CC2420_MANFIDL_MANFID = 0
};

enum cc2420_manfidh_enums {
  CC2420_MANFIDH_VERSION = 12, 
  CC2420_MANFIDH_PARTNUM = 0
};

enum cc2420_fsmtc_enums {
  CC2420_FSMTC_TC_RXCHAIN2RX = 13, 
  CC2420_FSMTC_TC_SWITCH2TX = 10, 
  CC2420_FSMTC_TC_PAON2TX = 6, 
  CC2420_FSMTC_TC_TXEND2SWITCH = 3, 
  CC2420_FSMTC_TC_TXEND2PAOFF = 0
};

enum cc2420_sfdmux_enums {
  CC2420_SFDMUX_SFD = 0, 
  CC2420_SFDMUX_XOSC16M_STABLE = 24
};

enum cc2420_security_enums {
  CC2420_NO_SEC = 0, 
  CC2420_CBC_MAC = 1, 
  CC2420_CTR = 2, 
  CC2420_CCM = 3, 
  NO_SEC = 0, 
  CBC_MAC_4 = 1, 
  CBC_MAC_8 = 2, 
  CBC_MAC_16 = 3, 
  CTR = 4, 
  CCM_4 = 5, 
  CCM_8 = 6, 
  CCM_16 = 7
};


enum __nesc_unnamed4264 {

  CC2420_INVALID_TIMESTAMP = 0x80000000L
};
# 6 "/opt/tinyos-2.1.2/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4265 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4266 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 83 "/opt/tinyos-2.1.2/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4267 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4268 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4269 {
  SERIAL_PROTO_ACK = 67, 
  SERIAL_PROTO_PACKET_ACK = 68, 
  SERIAL_PROTO_PACKET_NOACK = 69, 
  SERIAL_PROTO_PACKET_UNKNOWN = 255
};
#line 121
#line 109
typedef struct radio_stats {
  uint8_t version;
  uint8_t flags;
  uint8_t reserved;
  uint8_t platform;
  uint16_t MTU;
  uint16_t radio_crc_fail;
  uint16_t radio_queue_drops;
  uint16_t serial_crc_fail;
  uint16_t serial_tx_fail;
  uint16_t serial_short_packets;
  uint16_t serial_proto_drops;
} radio_stats_t;







#line 123
typedef nx_struct serial_header {
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t length;
  nx_am_group_t group;
  nx_am_id_t type;
} __attribute__((packed)) serial_header_t;




#line 131
typedef nx_struct serial_packet {
  serial_header_t header;
  nx_uint8_t data[];
} __attribute__((packed)) serial_packet_t;



#line 136
typedef nx_struct serial_metadata {
  nx_uint8_t ack;
} __attribute__((packed)) serial_metadata_t;
# 59 "/opt/tinyos-2.1.2/tos/platforms/telosa/platform_message.h"
#line 56
typedef union message_header {
  cc2420_header_t cc2420;
  serial_header_t serial;
} message_header_t;



#line 61
typedef union TOSRadioFooter {
  cc2420_footer_t cc2420;
} message_footer_t;




#line 65
typedef union TOSRadioMetadata {
  cc2420_metadata_t cc2420;
  serial_metadata_t serial;
} message_metadata_t;
# 19 "/opt/tinyos-2.1.2/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[28];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 31 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayer.h"
#line 28
typedef nx_struct cc2420x_header_t {

  nxle_uint8_t length;
} __attribute__((packed)) cc2420x_header_t;









#line 33
typedef struct cc2420x_metadata_t {

  uint8_t lqi;
  union  {

    uint8_t power;
    uint8_t rssi;
  } ;
} cc2420x_metadata_t;

enum cc2420X_timing_enums {
  CC2420X_SYMBOL_TIME = 16, 
  IDLE_2_RX_ON_TIME = 12 * CC2420X_SYMBOL_TIME, 
  PD_2_IDLE_TIME = 860, 
  STROBE_TO_TX_ON_TIME = 12 * CC2420X_SYMBOL_TIME, 






  TX_SFD_DELAY = STROBE_TO_TX_ON_TIME + 10 * CC2420X_SYMBOL_TIME - 25, 

  RX_SFD_DELAY = 0
};

enum cc2420X_reg_access_enums {
  CC2420X_CMD_REGISTER_MASK = 0x3f, 
  CC2420X_CMD_REGISTER_READ = 0x40, 
  CC2420X_CMD_REGISTER_WRITE = 0x00, 
  CC2420X_CMD_TXRAM_WRITE = 0x80
};
#line 79
#line 66
typedef union cc2420X_status {
  uint16_t value;
  struct  {
    unsigned reserved0 : 1;
    unsigned rssi_valid : 1;
    unsigned lock : 1;
    unsigned tx_active : 1;

    unsigned enc_busy : 1;
    unsigned tx_underflow : 1;
    unsigned xosc16m_stable : 1;
    unsigned reserved7 : 1;
  } ;
} cc2420X_status_t;
#line 92
#line 81
typedef union cc2420X_iocfg0 {
  uint16_t value;
  struct __nesc_unnamed4270 {
    unsigned fifop_thr : 7;
    unsigned cca_polarity : 1;
    unsigned sfd_polarity : 1;
    unsigned fifop_polarity : 1;
    unsigned fifo_polarity : 1;
    unsigned bcn_accept : 1;
    unsigned reserved : 4;
  } f;
} cc2420X_iocfg0_t;


static const cc2420X_iocfg0_t cc2420X_iocfg0_default = { .f.fifop_thr = 64, .f.cca_polarity = 0, .f.sfd_polarity = 0, .f.fifop_polarity = 0, .f.fifo_polarity = 0, .f.bcn_accept = 0, .f.reserved = 0 };









#line 97
typedef union cc2420X_iocfg1 {
  uint16_t value;
  struct __nesc_unnamed4271 {
    unsigned ccamux : 5;
    unsigned sfdmux : 5;
    unsigned hssd_src : 3;
    unsigned reserved : 3;
  } f;
} cc2420X_iocfg1_t;
#line 119
#line 109
typedef union cc2420X_fsctrl {
  uint16_t value;
  struct __nesc_unnamed4272 {
    unsigned freq : 10;
    unsigned lock_status : 1;
    unsigned lock_length : 1;
    unsigned cal_running : 1;
    unsigned cal_done : 1;
    unsigned lock_thr : 2;
  } f;
} cc2420X_fsctrl_t;

static const cc2420X_fsctrl_t cc2420X_fsctrl_default = { .f.lock_thr = 1, .f.freq = 357, .f.lock_status = 0, .f.lock_length = 0, .f.cal_running = 0, .f.cal_done = 0 };
#line 136
#line 123
typedef union cc2420X_mdmctrl0 {
  uint16_t value;
  struct __nesc_unnamed4273 {
    unsigned preamble_length : 4;
    unsigned autoack : 1;
    unsigned autocrc : 1;
    unsigned cca_mode : 2;
    unsigned cca_hyst : 3;
    unsigned adr_decode : 1;
    unsigned pan_coordinator : 1;
    unsigned reserved_frame_mode : 1;
    unsigned reserved : 2;
  } f;
} cc2420X_mdmctrl0_t;

static const cc2420X_mdmctrl0_t cc2420X_mdmctrl0_default = { .f.preamble_length = 2, .f.autocrc = 1, .f.cca_mode = 3, .f.cca_hyst = 2, .f.adr_decode = 1 };
#line 151
#line 140
typedef union cc2420X_txctrl {
  uint16_t value;
  struct __nesc_unnamed4274 {
    unsigned pa_level : 5;
    unsigned reserved : 1;
    unsigned pa_current : 3;
    unsigned txmix_current : 2;
    unsigned txmix_caparray : 2;
    unsigned tx_turnaround : 1;
    unsigned txmixbuf_cur : 2;
  } f;
} cc2420X_txctrl_t;

static const cc2420X_txctrl_t cc2420X_txctrl_default = { .f.pa_level = 31, .f.reserved = 1, .f.pa_current = 3, .f.tx_turnaround = 1, .f.txmixbuf_cur = 2 };










enum __nesc_unnamed4275 {
  CC2420X_TX_PWR_MASK = 0x1f, 
  CC2420X_CHANNEL_MASK = 0x1f
};

enum cc2420X_config_reg_enums {
  CC2420X_SNOP = 0x00, 
  CC2420X_SXOSCON = 0x01, 
  CC2420X_STXCAL = 0x02, 
  CC2420X_SRXON = 0x03, 
  CC2420X_STXON = 0x04, 
  CC2420X_STXONCCA = 0x05, 
  CC2420X_SRFOFF = 0x06, 
  CC2420X_SXOSCOFF = 0x07, 
  CC2420X_SFLUSHRX = 0x08, 
  CC2420X_SFLUSHTX = 0x09, 
  CC2420X_SACK = 0x0a, 
  CC2420X_SACKPEND = 0x0b, 
  CC2420X_SRXDEC = 0x0c, 
  CC2420X_STXENC = 0x0d, 
  CC2420X_SAES = 0x0e, 
  CC2420X_MAIN = 0x10, 
  CC2420X_MDMCTRL0 = 0x11, 
  CC2420X_MDMCTRL1 = 0x12, 
  CC2420X_RSSI = 0x13, 
  CC2420X_SYNCWORD = 0x14, 
  CC2420X_TXCTRL = 0x15, 
  CC2420X_RXCTRL0 = 0x16, 
  CC2420X_RXCTRL1 = 0x17, 
  CC2420X_FSCTRL = 0x18, 
  CC2420X_SECCTRL0 = 0x19, 
  CC2420X_SECCTRL1 = 0x1a, 
  CC2420X_BATTMON = 0x1b, 
  CC2420X_IOCFG0 = 0x1c, 
  CC2420X_IOCFG1 = 0x1d, 
  CC2420X_MANFIDL = 0x1e, 
  CC2420X_MANFIDH = 0x1f, 
  CC2420X_FSMTC = 0x20, 
  CC2420X_MANAND = 0x21, 
  CC2420X_MANOR = 0x22, 
  CC2420X_AGCCTRL = 0x23, 
  CC2420X_AGCTST0 = 0x24, 
  CC2420X_AGCTST1 = 0x25, 
  CC2420X_AGCTST2 = 0x26, 
  CC2420X_FSTST0 = 0x27, 
  CC2420X_FSTST1 = 0x28, 
  CC2420X_FSTST2 = 0x29, 
  CC2420X_FSTST3 = 0x2a, 
  CC2420X_RXBPFTST = 0x2b, 
  CC2420X_FSMSTATE = 0x2c, 
  CC2420X_ADCTST = 0x2d, 
  CC2420X_DACTST = 0x2e, 
  CC2420X_TOPTST = 0x2f, 
  CC2420X_TXFIFO = 0x3e, 
  CC2420X_RXFIFO = 0x3f
};

enum cc2420X_ram_addr_enums {
  CC2420X_RAM_TXFIFO = 0x000, 
  CC2420X_RAM_TXFIFO_END = 0x7f, 
  CC2420X_RAM_RXFIFO = 0x080, 
  CC2420X_RAM_KEY0 = 0x100, 
  CC2420X_RAM_RXNONCE = 0x110, 
  CC2420X_RAM_SABUF = 0x120, 
  CC2420X_RAM_KEY1 = 0x130, 
  CC2420X_RAM_TXNONCE = 0x140, 
  CC2420X_RAM_CBCSTATE = 0x150, 
  CC2420X_RAM_IEEEADR = 0x160, 
  CC2420X_RAM_PANID = 0x168, 
  CC2420X_RAM_SHORTADR = 0x16a
};
# 49 "/opt/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/RadioConfig.h"
typedef TMicro TRadio;
typedef uint16_t tradio_size;
# 40 "/opt/tinyos-2.1.2/tos/types/IeeeEui64.h"
enum __nesc_unnamed4276 {
#line 40
  IEEE_EUI64_LENGTH = 8
};


#line 42
typedef struct ieee_eui64 {
  uint8_t data[IEEE_EUI64_LENGTH];
} ieee_eui64_t;
# 47 "/opt/tinyos-2.1.2/tos/types/Ieee154.h"
typedef uint16_t ieee154_panid_t;
typedef uint16_t ieee154_saddr_t;
typedef ieee_eui64_t ieee154_laddr_t;







#line 51
typedef struct __nesc_unnamed4277 {
  uint8_t ieee_mode : 2;
  union __nesc_unnamed4278 {
    ieee154_saddr_t saddr;
    ieee154_laddr_t laddr;
  } ieee_addr;
} ieee154_addr_t;



enum __nesc_unnamed4279 {
  IEEE154_BROADCAST_ADDR = 0xffff, 
  IEEE154_LINK_MTU = 127
};

struct ieee154_frame_addr {
  ieee154_addr_t ieee_src;
  ieee154_addr_t ieee_dst;
  ieee154_panid_t ieee_dstpan;
};

enum __nesc_unnamed4280 {
  IEEE154_MIN_HDR_SZ = 6
};
#line 86
enum ieee154_fcf_enums {
  IEEE154_FCF_FRAME_TYPE = 0, 
  IEEE154_FCF_SECURITY_ENABLED = 3, 
  IEEE154_FCF_FRAME_PENDING = 4, 
  IEEE154_FCF_ACK_REQ = 5, 
  IEEE154_FCF_INTRAPAN = 6, 
  IEEE154_FCF_DEST_ADDR_MODE = 10, 
  IEEE154_FCF_SRC_ADDR_MODE = 14
};

enum ieee154_fcf_type_enums {
  IEEE154_TYPE_BEACON = 0, 
  IEEE154_TYPE_DATA = 1, 
  IEEE154_TYPE_ACK = 2, 
  IEEE154_TYPE_MAC_CMD = 3, 
  IEEE154_TYPE_MASK = 7
};

enum ieee154_fcf_addr_mode_enums {
  IEEE154_ADDR_NONE = 0, 
  IEEE154_ADDR_SHORT = 2, 
  IEEE154_ADDR_EXT = 3, 
  IEEE154_ADDR_MASK = 3
};
# 45 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TinyosNetworkLayer.h"
#line 42
typedef nx_struct network_header_t {

  nxle_uint8_t network;
} __attribute__((packed)) network_header_t;
# 47 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.h"
#line 40
typedef nx_struct ieee154_simple_header_t {

  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;
} __attribute__((packed)) ieee154_simple_header_t;
# 43 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayer.h"
#line 40
typedef nx_struct activemessage_header_t {

  nx_am_id_t type;
} __attribute__((packed)) activemessage_header_t;
# 42 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MetadataFlagsLayer.h"
#line 38
typedef struct flags_metadata_t {


  uint8_t flags;
} flags_metadata_t;
# 41 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TimeStampingLayer.h"
#line 38
typedef struct timestamp_metadata_t {

  uint32_t timestamp;
} timestamp_metadata_t;
# 41 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/LowPowerListeningLayer.h"
#line 38
typedef struct lpl_metadata_t {

  uint16_t sleepint;
} lpl_metadata_t;
# 42 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/PacketLinkLayer.h"
#line 38
typedef struct link_metadata_t {

  uint16_t maxRetries;
  uint16_t retryDelay;
} link_metadata_t;
# 47 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadio.h"
#line 37
typedef nx_struct cc2420xpacket_header_t {

  cc2420x_header_t cc2420x;
  ieee154_simple_header_t ieee154;

  network_header_t network;


  activemessage_header_t am;
} __attribute__((packed)) 
cc2420xpacket_header_t;




#line 49
typedef nx_struct cc2420xpacket_footer_t {
} __attribute__((packed)) 

cc2420xpacket_footer_t;
#line 65
#line 54
typedef struct cc2420xpacket_metadata_t {







  timestamp_metadata_t timestamp;
  flags_metadata_t flags;
  cc2420x_metadata_t cc2420x;
} cc2420xpacket_metadata_t;
# 33 "/opt/tinyos-2.1.2/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
# 45 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TimeSyncMessageLayer.h"
typedef nx_int32_t timesync_relative_t;


typedef nx_uint32_t timesync_absolute_t;









#line 50
typedef nx_struct timesync_footer_t {

  nx_am_id_t type;
  nx_union timestamp_t {

    timesync_relative_t relative;
    timesync_absolute_t absolute;
  } __attribute__((packed)) timestamp;
} __attribute__((packed)) timesync_footer_t;
# 56 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/msp430usart.h"
#line 48
typedef enum __nesc_unnamed4281 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;










#line 58
typedef struct __nesc_unnamed4282 {
  unsigned int swrst : 1;
  unsigned int mm : 1;
  unsigned int sync : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
} __attribute((packed))  msp430_uctl_t;









#line 69
typedef struct __nesc_unnamed4283 {
  unsigned int txept : 1;
  unsigned int stc : 1;
  unsigned int txwake : 1;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
} __attribute((packed))  msp430_utctl_t;










#line 79
typedef struct __nesc_unnamed4284 {
  unsigned int rxerr : 1;
  unsigned int rxwake : 1;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int brk : 1;
  unsigned int oe : 1;
  unsigned int pe : 1;
  unsigned int fe : 1;
} __attribute((packed))  msp430_urctl_t;
#line 116
#line 99
typedef struct __nesc_unnamed4285 {
  unsigned int ubr : 16;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int  : 3;

  unsigned int  : 1;
  unsigned int stc : 1;
  unsigned int  : 2;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
  unsigned int  : 0;
} msp430_spi_config_t;





#line 118
typedef struct __nesc_unnamed4286 {
  uint16_t ubr;
  uint8_t uctl;
  uint8_t utctl;
} msp430_spi_registers_t;




#line 124
typedef union __nesc_unnamed4287 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;

msp430_spi_union_config_t msp430_spi_default_config = { 
{ 
.ubr = 0x0002, 
.ssel = 0x02, 
.clen = 1, 
.listen = 0, 
.mm = 1, 
.ckph = 1, 
.ckpl = 0, 
.stc = 1 } };
#line 169
#line 150
typedef enum __nesc_unnamed4288 {

  UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x94, 
  UBR_32KHZ_1800 = 0x0012, UMCTL_32KHZ_1800 = 0x84, 
  UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x6D, 
  UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x77, 
  UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x29, 

  UBR_1MHZ_1200 = 0x0369, UMCTL_1MHZ_1200 = 0x7B, 
  UBR_1MHZ_1800 = 0x0246, UMCTL_1MHZ_1800 = 0x55, 
  UBR_1MHZ_2400 = 0x01B4, UMCTL_1MHZ_2400 = 0xDF, 
  UBR_1MHZ_4800 = 0x00DA, UMCTL_1MHZ_4800 = 0xAA, 
  UBR_1MHZ_9600 = 0x006D, UMCTL_1MHZ_9600 = 0x44, 
  UBR_1MHZ_19200 = 0x0036, UMCTL_1MHZ_19200 = 0xB5, 
  UBR_1MHZ_38400 = 0x001B, UMCTL_1MHZ_38400 = 0x94, 
  UBR_1MHZ_57600 = 0x0012, UMCTL_1MHZ_57600 = 0x84, 
  UBR_1MHZ_76800 = 0x000D, UMCTL_1MHZ_76800 = 0x6D, 
  UBR_1MHZ_115200 = 0x0009, UMCTL_1MHZ_115200 = 0x10, 
  UBR_1MHZ_230400 = 0x0004, UMCTL_1MHZ_230400 = 0x55
} msp430_uart_rate_t;
#line 200
#line 171
typedef struct __nesc_unnamed4289 {
  unsigned int ubr : 16;

  unsigned int umctl : 8;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
  unsigned int  : 0;

  unsigned int  : 3;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int  : 1;

  unsigned int  : 2;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int  : 4;
  unsigned int  : 0;

  unsigned int utxe : 1;
  unsigned int urxe : 1;
} msp430_uart_config_t;








#line 202
typedef struct __nesc_unnamed4290 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl;
  uint8_t utctl;
  uint8_t urctl;
  uint8_t ume;
} msp430_uart_registers_t;




#line 211
typedef union __nesc_unnamed4291 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;
#line 248
#line 240
typedef struct __nesc_unnamed4292 {
  unsigned int i2cstt : 1;
  unsigned int i2cstp : 1;
  unsigned int i2cstb : 1;
  unsigned int i2cctrx : 1;
  unsigned int i2cssel : 2;
  unsigned int i2ccrm : 1;
  unsigned int i2cword : 1;
} __attribute((packed))  msp430_i2ctctl_t;
#line 276
#line 253
typedef struct __nesc_unnamed4293 {
  unsigned int  : 1;
  unsigned int mst : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int xa : 1;
  unsigned int  : 1;
  unsigned int txdmaen : 1;
  unsigned int rxdmaen : 1;

  unsigned int  : 4;
  unsigned int i2cssel : 2;
  unsigned int i2crm : 1;
  unsigned int i2cword : 1;

  unsigned int i2cpsc : 8;

  unsigned int i2csclh : 8;

  unsigned int i2cscll : 8;

  unsigned int i2coa : 10;
  unsigned int  : 6;
} msp430_i2c_config_t;








#line 278
typedef struct __nesc_unnamed4294 {
  uint8_t uctl;
  uint8_t i2ctctl;
  uint8_t i2cpsc;
  uint8_t i2csclh;
  uint8_t i2cscll;
  uint16_t i2coa;
} msp430_i2c_registers_t;




#line 287
typedef union __nesc_unnamed4295 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
#line 309
typedef uint8_t uart_speed_t;
typedef uint8_t uart_parity_t;
typedef uint8_t uart_duplex_t;

enum __nesc_unnamed4296 {
  TOS_UART_1200 = 0, 
  TOS_UART_1800 = 1, 
  TOS_UART_2400 = 2, 
  TOS_UART_4800 = 3, 
  TOS_UART_9600 = 4, 
  TOS_UART_19200 = 5, 
  TOS_UART_38400 = 6, 
  TOS_UART_57600 = 7, 
  TOS_UART_76800 = 8, 
  TOS_UART_115200 = 9, 
  TOS_UART_230400 = 10
};

enum __nesc_unnamed4297 {
  TOS_UART_OFF, 
  TOS_UART_RONLY, 
  TOS_UART_TONLY, 
  TOS_UART_DUPLEX
};

enum __nesc_unnamed4298 {
  TOS_UART_PARITY_NONE, 
  TOS_UART_PARITY_EVEN, 
  TOS_UART_PARITY_ODD
};
# 40 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounter.h"
#line 38
typedef struct T2ghz {
  uint8_t dummy;
} T2ghz;
typedef TMilli BlinkToRadioC__Timer0__precision_tag;
typedef TMicro BlinkToRadioC__PacketTimeStamp__precision_tag;
typedef uint32_t BlinkToRadioC__PacketTimeStamp__size_type;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4299 {
  Msp430Timer32khzC__0__ALARM_ID = 0U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type;
typedef TRadio CC2420XRadioP__PacketTimeStamp__precision_tag;
typedef uint32_t CC2420XRadioP__PacketTimeStamp__size_type;
typedef TRadio /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__precision_tag;
typedef tradio_size /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__size_type;
typedef uint16_t RandomMlcgC__SeedInit__parameter;
typedef TRadio /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__PacketTimeStampRadio__precision_tag;
typedef uint32_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__PacketTimeStampRadio__size_type;
typedef TRadio /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__LocalTimeRadio__precision_tag;
typedef TMilli /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__LocalTimeMilli__precision_tag;
typedef TMilli /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__PacketTimeStampMilli__precision_tag;
typedef uint32_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__PacketTimeStampMilli__size_type;
typedef TMicro CC2420XDriverLayerP__BusyWait__precision_tag;
typedef uint16_t CC2420XDriverLayerP__BusyWait__size_type;
typedef uint8_t CC2420XDriverLayerP__PacketRSSI__value_type;
typedef TRadio CC2420XDriverLayerP__PacketTimeStamp__precision_tag;
typedef uint32_t CC2420XDriverLayerP__PacketTimeStamp__size_type;
typedef TRadio CC2420XDriverLayerP__LocalTime__precision_tag;
typedef uint8_t CC2420XDriverLayerP__PacketTransmitPower__value_type;
typedef uint8_t CC2420XDriverLayerP__PacketTimeSyncOffset__value_type;
typedef uint8_t CC2420XDriverLayerP__PacketLinkQuality__value_type;
typedef TMicro /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__precision_tag;
typedef uint16_t /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__precision_tag /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__precision_tag;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__size_type;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__precision_tag /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__precision_tag;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__size_type;
typedef TMicro /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__frequency_tag;
typedef /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__frequency_tag /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__precision_tag;
typedef uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__size_type;
enum /*HplCC2420XC.SpiC*/Msp430Spi0C__0____nesc_unnamed4300 {
  Msp430Spi0C__0__CLIENT_ID = 0U
};
enum /*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0____nesc_unnamed4301 {
  Msp430Usart0C__0__CLIENT_ID = 0U
};
enum /*HplCC2420XC.AlarmC.Msp430Timer*/Msp430TimerMicroC__0____nesc_unnamed4302 {
  Msp430TimerMicroC__0__ALARM_ID = 0U
};
typedef TMicro /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__frequency_tag;
typedef /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__frequency_tag /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__precision_tag;
typedef uint16_t /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type;
typedef TMicro Msp430HybridAlarmCounterP__CounterMicro__precision_tag;
typedef uint16_t Msp430HybridAlarmCounterP__CounterMicro__size_type;
typedef T2ghz Msp430HybridAlarmCounterP__Counter2ghz__precision_tag;
typedef uint32_t Msp430HybridAlarmCounterP__Counter2ghz__size_type;
typedef T32khz Msp430HybridAlarmCounterP__Alarm32khz__precision_tag;
typedef uint16_t Msp430HybridAlarmCounterP__Alarm32khz__size_type;
typedef T2ghz Msp430HybridAlarmCounterP__Alarm2ghz__precision_tag;
typedef uint32_t Msp430HybridAlarmCounterP__Alarm2ghz__size_type;
typedef TMicro Msp430HybridAlarmCounterP__AlarmMicro__precision_tag;
typedef uint16_t Msp430HybridAlarmCounterP__AlarmMicro__size_type;
typedef T32khz Msp430HybridAlarmCounterP__Counter32khz__precision_tag;
typedef uint16_t Msp430HybridAlarmCounterP__Counter32khz__size_type;
enum /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Timer*/Msp430Timer32khzC__1____nesc_unnamed4303 {
  Msp430Timer32khzC__1__ALARM_ID = 1U
};
typedef T32khz /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__frequency_tag;
typedef /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__frequency_tag /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__precision_tag;
typedef uint16_t /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__size_type;
enum /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Timer*/Msp430TimerMicroC__1____nesc_unnamed4304 {
  Msp430TimerMicroC__1__ALARM_ID = 1U
};
typedef TMicro /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__frequency_tag;
typedef /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__frequency_tag /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__precision_tag;
typedef uint16_t /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__size_type;
typedef TMicro /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_precision_tag;
typedef uint32_t /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_size_type;
typedef T2ghz /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__from_precision_tag;
typedef uint32_t /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__from_size_type;
typedef uint32_t /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__upper_count_type;
typedef /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__from_precision_tag /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__precision_tag;
typedef /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__from_size_type /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__size_type;
typedef /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_precision_tag /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__precision_tag;
typedef /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_size_type /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__size_type;
typedef TMicro /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag;
typedef /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__precision_tag;
typedef /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__precision_tag;
typedef uint32_t /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type;
enum AMQueueP____nesc_unnamed4305 {
  AMQueueP__NUM_CLIENTS = 1U
};
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
#line 62
static error_t MotePlatformC__Init__init(void );
# 46 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 43
static void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );



static void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 42
static void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );





static void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 45
static void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 40
static void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__default__initClocks(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t Msp430ClockP__Init__init(void );
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(
# 51 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x2b395a188458);
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__isOverflowPending(void );
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 51 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x2b395a188458);
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
# 44 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__enableEvents(void );
#line 47
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__clearPendingInterrupt(void );
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 41 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEventFromNow(uint16_t delta);
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 44 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 58
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__disableEvents(void );
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 44 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 44 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );
#line 47
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 41 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t delta);
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 44 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 68
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void );
# 55 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm);
#line 42
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void );
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 44 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );

static bool /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__areEventsEnabled(void );
#line 58
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );
# 41 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t delta);
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 44 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 44 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 44 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 44 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 76 "/opt/tinyos-2.1.2/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 56 "/opt/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2b395a090650);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 56 "/opt/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2b395a090650);
# 57 "/opt/tinyos-2.1.2/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 72
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 65
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 67 "/opt/tinyos-2.1.2/tos/interfaces/Leds.nc"
static void LedsP__Leds__led0Toggle(void );
#line 83
static void LedsP__Leds__led1Toggle(void );
#line 100
static void LedsP__Leds__led2Toggle(void );
# 78 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__makeInput(void );
#line 73
static bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void );
#line 78
static void /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__makeInput(void );
#line 73
static bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void );
#line 78
static void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void );
#line 73
static bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void );
#line 99
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 99
static void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void );
#line 73
static bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void );
#line 99
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void );
#line 92
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void );
#line 85
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void );




static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void );
#line 85
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void );




static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void );




static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle(void );
#line 85
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );









static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle(void );
#line 85
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );









static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle(void );
#line 85
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );
# 42 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle(void );



static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );

static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle(void );



static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );

static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void );



static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
# 83 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static void BlinkToRadioC__Timer0__fired(void );
# 113 "/opt/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
static void BlinkToRadioC__AMControl__startDone(error_t error);
#line 138
static void BlinkToRadioC__AMControl__stopDone(error_t error);
# 60 "/opt/tinyos-2.1.2/tos/interfaces/Boot.nc"
static void BlinkToRadioC__Boot__booted(void );
# 110 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static void BlinkToRadioC__AMSend__sendDone(
#line 103
message_t * msg, 






error_t error);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



BlinkToRadioC__Receive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 103 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
#line 64
static /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
# 109 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );
# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
# 136 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
#line 129
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
# 83 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );
#line 83
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(
# 48 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b395a6cd5d8);
# 73 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(
# 48 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b395a6cd5d8, 
# 73 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
uint32_t dt);
# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 63 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageConfig.nc"
static error_t CC2420XRadioP__ActiveMessageConfig__checkFrame(message_t *msg);
#line 40
static am_addr_t CC2420XRadioP__ActiveMessageConfig__destination(message_t *msg);








static void CC2420XRadioP__ActiveMessageConfig__setSource(message_t *msg, am_addr_t addr);
#line 43
static void CC2420XRadioP__ActiveMessageConfig__setDestination(message_t *msg, am_addr_t addr);
#line 55
static void CC2420XRadioP__ActiveMessageConfig__setGroup(message_t *msg, am_group_t grp);
# 60 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void CC2420XRadioP__RadioAlarm__fired(void );
# 35 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverConfig.nc"
static uint8_t CC2420XRadioP__CC2420XDriverConfig__maxPayloadLength(void );
#line 29
static uint8_t CC2420XRadioP__CC2420XDriverConfig__headerLength(message_t *msg);
#line 41
static uint8_t CC2420XRadioP__CC2420XDriverConfig__metadataLength(message_t *msg);






static uint8_t CC2420XRadioP__CC2420XDriverConfig__headerPreloadLength(void );





static bool CC2420XRadioP__CC2420XDriverConfig__requiresRssiCca(message_t *msg);
# 86 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
static void CC2420XRadioP__SoftwareAckConfig__reportChannelError(void );
#line 80
static void CC2420XRadioP__SoftwareAckConfig__createAckPacket(message_t *data, message_t *ack);
#line 55
static bool CC2420XRadioP__SoftwareAckConfig__requiresAckWait(message_t *msg);






static bool CC2420XRadioP__SoftwareAckConfig__isAckPacket(message_t *msg);






static bool CC2420XRadioP__SoftwareAckConfig__verifyAckPacket(message_t *data, message_t *ack);
#line 43
static uint16_t CC2420XRadioP__SoftwareAckConfig__getAckTimeout(void );
#line 75
static bool CC2420XRadioP__SoftwareAckConfig__requiresAckReply(message_t *msg);
# 52 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/UniqueConfig.nc"
static void CC2420XRadioP__UniqueConfig__setSequenceNumber(message_t *msg, uint8_t number);





static void CC2420XRadioP__UniqueConfig__reportChannelError(void );
#line 42
static uint8_t CC2420XRadioP__UniqueConfig__getSequenceNumber(message_t *msg);




static am_addr_t CC2420XRadioP__UniqueConfig__getSender(message_t *msg);
# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/RandomCollisionConfig.nc"
static uint16_t CC2420XRadioP__RandomCollisionConfig__getCongestionBackoff(message_t *msg);
#line 40
static uint16_t CC2420XRadioP__RandomCollisionConfig__getInitialBackoff(message_t *msg);










static uint16_t CC2420XRadioP__RandomCollisionConfig__getMinimumBackoff(void );





static uint16_t CC2420XRadioP__RandomCollisionConfig__getTransmitBarrier(message_t *msg);
# 60 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__default__fired(
# 43 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
uint8_t arg_0x2b395a8ab220);
# 50 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__wait(
# 43 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
uint8_t arg_0x2b395a8ab220, 
# 50 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
tradio_size timeout);




static void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__cancel(
# 43 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
uint8_t arg_0x2b395a8ab220);
# 45 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static bool /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__isFree(
# 43 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
uint8_t arg_0x2b395a8ab220);
# 65 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static tradio_size /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__getNow(
# 43 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
uint8_t arg_0x2b395a8ab220);
# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__fired(void );
# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
static void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Tasklet__run(void );










static void TaskletC__Tasklet__schedule(void );
#line 72
static void TaskletC__Tasklet__suspend(void );






static void TaskletC__Tasklet__resume(void );
# 42 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareReceive.nc"
static message_t */*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubReceive__receive(message_t *msg);
# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubSend__sendDone(message_t *msg, error_t error);
# 59 "/opt/tinyos-2.1.2/tos/interfaces/SendNotifier.nc"
static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SendNotifier__default__aboutToSend(
# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
am_id_t arg_0x2b395a917df8, 
# 59 "/opt/tinyos-2.1.2/tos/interfaces/SendNotifier.nc"
am_addr_t dest, 
#line 57
message_t * msg);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Packet.nc"
static uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Packet__payloadLength(
#line 74
message_t * msg);
#line 106
static uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Packet__maxPayloadLength(void );
#line 94
static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Packet__setPayloadLength(
#line 90
message_t * msg, 



uint8_t len);
# 80 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static error_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMSend__send(
# 45 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
am_id_t arg_0x2b395a91a5f0, 
# 80 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Snoop__default__receive(
# 47 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
am_id_t arg_0x2b395a9172f0, 
# 71 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
static uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__RadioPacket__payloadLength(message_t *msg);









static uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__RadioPacket__maxPayloadLength(void );
#line 54
static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 43
static uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__RadioPacket__headerLength(message_t *msg);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Receive__default__receive(
# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
am_id_t arg_0x2b395a919780, 
# 71 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
#line 78
static 
#line 74
message_t * 



/*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__ReceiveDefault__default__receive(
# 51 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
am_id_t arg_0x2b395a916a38, 
# 71 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 68 "/opt/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
static am_addr_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__address(void );









static am_addr_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__destination(
#line 74
message_t * amsg);
#line 121
static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__setSource(
#line 117
message_t * amsg, 



am_addr_t addr);
#line 103
static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__setDestination(
#line 99
message_t * amsg, 



am_addr_t addr);
#line 147
static am_id_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__type(
#line 143
message_t * amsg);
#line 162
static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__setType(
#line 158
message_t * amsg, 



am_id_t t);
#line 136
static bool /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__isForMe(
#line 133
message_t * amsg);
#line 187
static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__setGroup(
#line 184
message_t * amsg, 


am_group_t grp);







static am_group_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__localGroup(void );
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SnoopDefault__default__receive(
# 52 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
am_id_t arg_0x2b395a9156b8, 
# 71 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 55 "/opt/tinyos-2.1.2/tos/system/ActiveMessageAddressC.nc"
static am_addr_t ActiveMessageAddressC__amAddress(void );
# 50 "/opt/tinyos-2.1.2/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );




static am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
static void /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__SubSend__sendDone(message_t *msg, error_t error);
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__granted(void );
# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
static error_t /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__BareSend__send(message_t *msg);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__Init__init(void );
# 79 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
static error_t /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
#line 53
static bool /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );








static bool /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 53 "/opt/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(
# 52 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b395a9fe1a0);
# 61 "/opt/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__immediateRequested(
# 52 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b395a9fe1a0);
# 65 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(
# 56 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b395a9fd6e0);
# 59 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(
# 56 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b395a9fd6e0);
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__release(
# 51 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b395a9be020);
# 97 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__immediateRequest(
# 51 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b395a9be020);
# 88 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__request(
# 51 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b395a9be020);
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__default__granted(
# 51 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b395a9be020);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__runTask(void );
# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
static void /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__SubSend__sendDone(message_t *msg, error_t error);
# 42 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareReceive.nc"
static message_t */*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__SubReceive__receive(message_t *msg);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Packet.nc"
static uint8_t /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Packet__payloadLength(
#line 74
message_t * msg);
# 97 "/opt/tinyos-2.1.2/tos/interfaces/Ieee154Send.nc"
static void /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Ieee154Send__default__sendDone(message_t *msg, error_t error);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Ieee154Receive__default__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
static void /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubSend__sendDone(message_t *msg, error_t error);
# 42 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareReceive.nc"
static message_t */*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubReceive__receive(message_t *msg);
# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
static uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__Ieee154Packet__payloadLength(message_t *msg);
#line 43
static uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__Ieee154Packet__headerLength(message_t *msg);
# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
static error_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosSend__send(message_t *msg);
# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
static uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosPacket__payloadLength(message_t *msg);









static uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosPacket__maxPayloadLength(void );
#line 54
static void /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 43
static uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosPacket__headerLength(message_t *msg);





static uint8_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__RadioPacket__payloadLength(message_t *msg);









static uint8_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__RadioPacket__maxPayloadLength(void );
#line 54
static void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 43
static uint8_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__RadioPacket__headerLength(message_t *msg);
# 131 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
static uint16_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDestPan(message_t *msg);
#line 120
static uint8_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDSN(message_t *msg);
#line 75
static bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isAckFrame(message_t *msg);
#line 156
static void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setSrcAddr(message_t *msg, uint16_t addr);
#line 69
static void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__createDataFrame(message_t *msg);
#line 125
static void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDSN(message_t *msg, uint8_t dsn);
#line 88
static void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__createAckReply(message_t *data, message_t *ack);
#line 151
static uint16_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getSrcAddr(message_t *msg);
#line 63
static bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isDataFrame(message_t *msg);
#line 99
static bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getAckRequired(message_t *msg);
#line 94
static bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__verifyAckReply(message_t *data, message_t *ack);
#line 178
static ieee154_saddr_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__localAddr(void );
#line 162
static bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__requiresAckWait(message_t *msg);










static ieee154_panid_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__localPan(void );
#line 146
static void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDestAddr(message_t *msg, uint16_t addr);
#line 136
static void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDestPan(message_t *msg, uint16_t pan);
#line 185
static bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isForMe(message_t *msg);
#line 168
static bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__requiresAckReply(message_t *msg);
#line 141
static uint16_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDestAddr(message_t *msg);
# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
static void /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__SubSend__sendDone(message_t *msg, error_t error);
# 53 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
static message_t */*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__SubReceive__receive(message_t *msg);
#line 46
static bool /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__SubReceive__header(message_t *msg);
# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
static error_t /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__Send__send(message_t *msg);
# 80 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/Neighborhood.nc"
static void /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__Neighborhood__evicted(uint8_t idx);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__Init__init(void );
# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/NeighborhoodFlag.nc"
static bool NeighborhoodP__NeighborhoodFlag__get(
# 43 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/NeighborhoodP.nc"
uint8_t arg_0x2b395ab36970, 
# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/NeighborhoodFlag.nc"
uint8_t idx);




static void NeighborhoodP__NeighborhoodFlag__set(
# 43 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/NeighborhoodP.nc"
uint8_t arg_0x2b395ab36970, 
# 51 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/NeighborhoodFlag.nc"
uint8_t idx);
# 71 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/Neighborhood.nc"
static uint8_t NeighborhoodP__Neighborhood__insertNode(am_addr_t id);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t NeighborhoodP__Init__init(void );
# 104 "/opt/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
static error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__SplitControl__start(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__SoftwareInit__init(void );
# 69 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioState.nc"
static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioState__done(void );
# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
static error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Send__send(message_t *msg);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__sendTask__runTask(void );
# 53 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
static message_t */*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioReceive__receive(message_t *msg);
#line 46
static bool /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioReceive__header(message_t *msg);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__stateDoneTask__runTask(void );
#line 75
static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__deliverTask__runTask(void );
# 63 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioSend__ready(void );
#line 56
static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioSend__sendDone(error_t error);
# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioChannel.nc"
static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioChannel__default__setChannelDone(void );
# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Tasklet__run(void );
# 63 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static void /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubSend__ready(void );
#line 56
static void /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubSend__sendDone(error_t error);
# 53 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
static message_t */*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubReceive__receive(message_t *msg);
#line 46
static bool /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubReceive__header(message_t *msg);
# 60 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__fired(void );
# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static error_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioSend__send(message_t *msg);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__calcNextRandom__runTask(void );
# 52 "/opt/tinyos-2.1.2/tos/interfaces/Random.nc"
static uint16_t RandomMlcgC__Random__rand16(void );
#line 46
static uint32_t RandomMlcgC__Random__rand32(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t RandomMlcgC__Init__init(void );
# 63 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubSend__ready(void );
#line 56
static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubSend__sendDone(error_t error);
# 53 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
static message_t */*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubReceive__receive(message_t *msg);
#line 46
static bool /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubReceive__header(message_t *msg);
# 60 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioAlarm__fired(void );
# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static error_t /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioSend__send(message_t *msg);
# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
static uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__payloadLength(message_t *msg);









static uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__maxPayloadLength(void );
#line 54
static void /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 43
static uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__headerLength(message_t *msg);
#line 65
static uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__metadataLength(message_t *msg);
# 63 "/opt/tinyos-2.1.2/tos/interfaces/PacketTimeStamp.nc"
static /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__PacketTimeStampRadio__size_type /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__PacketTimeStampRadio__timestamp(
#line 52
message_t * msg);
#line 78
static void /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__PacketTimeStampRadio__set(
#line 73
message_t * msg, 




/*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__PacketTimeStampRadio__size_type value);
# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
static uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__payloadLength(message_t *msg);









static uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__maxPayloadLength(void );
#line 54
static void /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 43
static uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__headerLength(message_t *msg);
#line 65
static uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__metadataLength(message_t *msg);
# 55 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
static void /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__PacketFlag__clear(
# 42 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MetadataFlagsLayerC.nc"
uint8_t arg_0x2b395ad1c260, 
# 55 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
message_t *msg);
#line 40
static bool /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__PacketFlag__get(
# 42 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MetadataFlagsLayerC.nc"
uint8_t arg_0x2b395ad1c260, 
# 40 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
message_t *msg);









static void /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__PacketFlag__set(
# 42 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MetadataFlagsLayerC.nc"
uint8_t arg_0x2b395ad1c260, 
# 50 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
message_t *msg);
# 57 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/PacketField.nc"
static void CC2420XDriverLayerP__PacketRSSI__set(message_t *msg, CC2420XDriverLayerP__PacketRSSI__value_type value);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t CC2420XDriverLayerP__SoftwareInit__init(void );
# 60 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void CC2420XDriverLayerP__RadioAlarm__fired(void );
# 56 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioState.nc"
static error_t CC2420XDriverLayerP__RadioState__turnOn(void );
# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/PacketField.nc"
static CC2420XDriverLayerP__PacketTransmitPower__value_type CC2420XDriverLayerP__PacketTransmitPower__get(message_t *msg);
#line 40
static bool CC2420XDriverLayerP__PacketTransmitPower__isSet(message_t *msg);
# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
static uint8_t CC2420XDriverLayerP__RadioPacket__payloadLength(message_t *msg);









static uint8_t CC2420XDriverLayerP__RadioPacket__maxPayloadLength(void );
#line 54
static void CC2420XDriverLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 43
static uint8_t CC2420XDriverLayerP__RadioPacket__headerLength(message_t *msg);
#line 65
static uint8_t CC2420XDriverLayerP__RadioPacket__metadataLength(message_t *msg);
# 68 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static void CC2420XDriverLayerP__FifopInterrupt__fired(void );
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void CC2420XDriverLayerP__releaseSpi__runTask(void );
# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/PacketField.nc"
static CC2420XDriverLayerP__PacketTimeSyncOffset__value_type CC2420XDriverLayerP__PacketTimeSyncOffset__get(message_t *msg);
#line 40
static bool CC2420XDriverLayerP__PacketTimeSyncOffset__isSet(message_t *msg);
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void CC2420XDriverLayerP__SpiResource__granted(void );
# 57 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/PacketField.nc"
static void CC2420XDriverLayerP__PacketLinkQuality__set(message_t *msg, CC2420XDriverLayerP__PacketLinkQuality__value_type value);
# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static error_t CC2420XDriverLayerP__RadioSend__send(message_t *msg);
# 61 "/opt/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
static void CC2420XDriverLayerP__SfdCapture__captured(uint16_t time);
# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
static void CC2420XDriverLayerP__Tasklet__run(void );
# 66 "/opt/tinyos-2.1.2/tos/lib/timer/BusyWait.nc"
static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__size_type dt);
# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void );
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__size_type /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get(void );
# 65 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(
# 76 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af8acc0);
# 59 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(
# 76 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af8acc0);
# 82 "/opt/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(
# 79 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af87ae8, 
# 75 "/opt/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(
# 82 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af85b40);
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(
# 81 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af86898);
# 97 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(
# 81 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af86898);
# 88 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(
# 81 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af86898);
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(
# 81 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af86898);
# 128 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(
# 81 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af86898);
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(
# 75 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af389f8);
# 97 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(
# 75 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af389f8);
# 88 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(
# 75 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af389f8);
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(
# 75 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af389f8);
# 128 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(
# 75 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af389f8);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/FastSpiByte.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitWrite(uint8_t data);
#line 74
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitReadWrite(uint8_t data);
#line 68
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitRead(void );
# 54 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void );
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void );
# 97 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430Usart0P__Usart__resetUsart(bool reset);
#line 179
static void HplMsp430Usart0P__Usart__disableIntr(void );
#line 90
static void HplMsp430Usart0P__Usart__setUmctl(uint8_t umctl);
#line 177
static void HplMsp430Usart0P__Usart__disableRxIntr(void );









static bool HplMsp430Usart0P__Usart__isTxIntrPending(void );
#line 207
static void HplMsp430Usart0P__Usart__clrIntr(void );
#line 80
static void HplMsp430Usart0P__Usart__setUbr(uint16_t ubr);
#line 224
static void HplMsp430Usart0P__Usart__tx(uint8_t data);
#line 128
static void HplMsp430Usart0P__Usart__disableUart(void );
#line 153
static void HplMsp430Usart0P__Usart__enableSpi(void );
#line 168
static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t HplMsp430Usart0P__Usart__rx(void );
#line 192
static bool HplMsp430Usart0P__Usart__isRxIntrPending(void );
#line 158
static void HplMsp430Usart0P__Usart__disableSpi(void );
# 54 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x2b395b0a50c8, 
# 54 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x2b395b0a50c8);
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void );
#line 39
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(
# 40 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x2b395b0a4020);
# 54 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );
# 79 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id);
#line 53
static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );








static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
# 53 "/opt/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(
# 55 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b395b0dc4b8);
# 61 "/opt/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(
# 55 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b395b0dc4b8);
# 65 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(
# 60 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b395b0da6e0);
# 59 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(
# 60 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b395b0da6e0);
# 56 "/opt/tinyos-2.1.2/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 73
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void );
#line 46
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void );
#line 81
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void );
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(
# 54 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b395b0dd2f0);
# 97 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(
# 54 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b395b0dd2f0);
# 88 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(
# 54 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b395b0dd2f0);
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(
# 54 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b395b0dd2f0);
# 128 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(
# 54 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b395b0dd2f0);
# 90 "/opt/tinyos-2.1.2/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
# 7 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430I2C0P__HplI2C__isI2C(void );
# 44 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void /*HplCC2420XC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void );
#line 43
static bool /*HplCC2420XC.CCAM*/Msp430GpioC__3__GeneralIO__get(void );


static void /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void );
#line 40
static void /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__set(void );
static void /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void );


static void /*HplCC2420XC.FIFOM*/Msp430GpioC__5__GeneralIO__makeInput(void );
#line 43
static bool /*HplCC2420XC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void );
static void /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__GeneralIO__makeInput(void );
#line 43
static bool /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void );


static void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void );
#line 40
static void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void );
static void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void );


static void /*HplCC2420XC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void );
#line 43
static bool /*HplCC2420XC.SFDM*/Msp430GpioC__8__GeneralIO__get(void );


static void /*HplCC2420XC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void );
#line 40
static void /*HplCC2420XC.VRENM*/Msp430GpioC__9__GeneralIO__set(void );
# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 54 "/opt/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
static error_t /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captureFallingEdge(void );
#line 66
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__disable(void );
#line 53
static error_t /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captureRisingEdge(void );
# 72 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 61 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__Interrupt__disable(void );
# 52 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__clear(void );
#line 72
static void HplMsp430InterruptP__Port14__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port26__clear(void );
#line 72
static void HplMsp430InterruptP__Port26__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port17__clear(void );
#line 72
static void HplMsp430InterruptP__Port17__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port21__clear(void );
#line 72
static void HplMsp430InterruptP__Port21__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port12__clear(void );
#line 72
static void HplMsp430InterruptP__Port12__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port24__clear(void );
#line 72
static void HplMsp430InterruptP__Port24__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port15__clear(void );
#line 72
static void HplMsp430InterruptP__Port15__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port27__clear(void );
#line 72
static void HplMsp430InterruptP__Port27__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port10__clear(void );
#line 47
static void HplMsp430InterruptP__Port10__disable(void );




static void HplMsp430InterruptP__Port22__clear(void );
#line 72
static void HplMsp430InterruptP__Port22__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port13__clear(void );
#line 72
static void HplMsp430InterruptP__Port13__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port25__clear(void );
#line 72
static void HplMsp430InterruptP__Port25__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port16__clear(void );
#line 72
static void HplMsp430InterruptP__Port16__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port20__clear(void );
#line 72
static void HplMsp430InterruptP__Port20__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port11__clear(void );
#line 72
static void HplMsp430InterruptP__Port11__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port23__clear(void );
#line 72
static void HplMsp430InterruptP__Port23__default__fired(void );
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 109 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__getNow(void );
#line 103
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(/*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type t0, /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type dt);
#line 66
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__start(/*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type dt);






static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Init__init(void );
# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void Msp430HybridAlarmCounterP__CounterMicro__overflow(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t Msp430HybridAlarmCounterP__McuPowerOverride__lowestState(void );
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static Msp430HybridAlarmCounterP__Counter2ghz__size_type Msp430HybridAlarmCounterP__Counter2ghz__get(void );






static bool Msp430HybridAlarmCounterP__Counter2ghz__isOverflowPending(void );
# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void Msp430HybridAlarmCounterP__Alarm32khz__fired(void );
#line 78
static void Msp430HybridAlarmCounterP__Alarm2ghz__default__fired(void );
#line 78
static void Msp430HybridAlarmCounterP__AlarmMicro__fired(void );
# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void Msp430HybridAlarmCounterP__Counter32khz__overflow(void );
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow(void );
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__fired(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__overflow(void );
# 103 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__startAt(/*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__size_type t0, /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__size_type dt);
#line 88
static bool /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__isRunning(void );
# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__overflow(void );
#line 64
static /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__size_type /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__get(void );
# 61 "/opt/tinyos-2.1.2/tos/lib/timer/LocalTime.nc"
static uint32_t /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get(void );
# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 80 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static error_t /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 100 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static void /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(
#line 96
message_t * msg, 



error_t error);
# 110 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(
# 48 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
am_id_t arg_0x2b395b468488, 
# 103 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(
# 46 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2b395b469318, 
# 67 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 100
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(
# 46 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2b395b469318, 
# 96 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );
#line 75
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t PlatformP__MoteInit__init(void );
#line 62
static error_t PlatformP__MoteClockInit__init(void );
#line 62
static error_t PlatformP__LedsInit__init(void );
# 10 "/opt/tinyos-2.1.2/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void );
# 6 "/opt/tinyos-2.1.2/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__uwait(uint16_t u);




static __inline void MotePlatformC__TOSH_wait(void );




static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set);










static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void );
#line 56
static inline error_t MotePlatformC__Init__init(void );
# 43 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__initTimerB(void );
#line 42
static void Msp430ClockP__Msp430ClockInit__initTimerA(void );
#line 40
static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__initClocks(void );
# 48 "/opt/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP__IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP__TACTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP__TAIV __asm ("0x012E");
static volatile uint16_t Msp430ClockP__TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP__TBIV __asm ("0x011E");

enum Msp430ClockP____nesc_unnamed4306 {

  Msp430ClockP__ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP__TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP__ACLK_CALIB_PERIOD
};

static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );



static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 76
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 97
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 112
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 127
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );





static inline void Msp430ClockP__startTimerA(void );
#line 160
static inline void Msp430ClockP__startTimerB(void );
#line 172
static void Msp430ClockP__set_dco_calib(int calib);





static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib);
#line 201
static inline void Msp430ClockP__busyCalibrateDco(void );
#line 226
static inline error_t Msp430ClockP__Init__init(void );
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(
# 51 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x2b395a188458);
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 62 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void );
#line 81
static inline bool /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__isOverflowPending(void );
#line 126
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 51 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x2b395a188458);
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 62 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
#line 126
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n);
# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time);
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void );
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__get(void );
# 55 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__compareControl(void );
#line 85
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControlAsCompare(void );
#line 130
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEventFromNow(uint16_t x);
#line 180
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 55 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t;


static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 135
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time);
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void );
# 55 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time);
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void );
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void );
# 55 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void );
#line 85
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );
#line 130
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x);
#line 180
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time);
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void );
# 55 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)  ;
#line 72
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(uint8_t l_cm);
#line 85
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void );
#line 110
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm);
#line 130
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 175
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void );




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
#line 192
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void );
# 55 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );
#line 130
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );




static inline bool /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__areEventsEnabled(void );









static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x);
#line 180
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time);
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void );
# 55 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time);
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void );
# 55 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time);
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void );
# 55 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time);
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void );
# 55 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerB1__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerA0__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerA1__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerB0__fired(void );
# 11 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x000C)))  ;
void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x000A)))  ;
void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x001A)))  ;
void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0018)))  ;
# 62 "/opt/tinyos-2.1.2/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 64 "/opt/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/McuSleepC.nc"
bool McuSleepC__dirty = TRUE;
mcu_power_t McuSleepC__powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC__msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC__getPowerState(void );
#line 117
static inline void McuSleepC__computePowerState(void );




static inline void McuSleepC__McuSleep__sleep(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
# 60 "/opt/tinyos-2.1.2/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
# 57 "/opt/tinyos-2.1.2/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
#line 72
static void RealMainP__Scheduler__taskLoop(void );
#line 65
static bool RealMainP__Scheduler__runNextTask(void );
# 63 "/opt/tinyos-2.1.2/tos/system/RealMainP.nc"
int main(void )   ;
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 56 "/opt/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2b395a090650);
# 76 "/opt/tinyos-2.1.2/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 61 "/opt/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4307 {

  SchedulerBasicP__NUM_TASKS = 12U, 
  SchedulerBasicP__NO_TASK = 255
};

uint8_t SchedulerBasicP__m_head;
uint8_t SchedulerBasicP__m_tail;
uint8_t SchedulerBasicP__m_next[SchedulerBasicP__NUM_TASKS];








static __inline uint8_t SchedulerBasicP__popTask(void );
#line 97
static inline bool SchedulerBasicP__isWaiting(uint8_t id);




static inline bool SchedulerBasicP__pushTask(uint8_t id);
#line 124
static inline void SchedulerBasicP__Scheduler__init(void );









static bool SchedulerBasicP__Scheduler__runNextTask(void );
#line 149
static inline void SchedulerBasicP__Scheduler__taskLoop(void );
#line 170
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id);




static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
# 42 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__toggle(void );



static void LedsP__Led0__makeOutput(void );
#line 40
static void LedsP__Led0__set(void );

static void LedsP__Led1__toggle(void );



static void LedsP__Led1__makeOutput(void );
#line 40
static void LedsP__Led1__set(void );

static void LedsP__Led2__toggle(void );



static void LedsP__Led2__makeOutput(void );
#line 40
static void LedsP__Led2__set(void );
# 56 "/opt/tinyos-2.1.2/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void );
#line 84
static inline void LedsP__Leds__led0Toggle(void );
#line 99
static inline void LedsP__Leds__led1Toggle(void );
#line 114
static inline void LedsP__Leds__led2Toggle(void );
# 59 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__makeInput(void );
#line 59
static inline uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__makeInput(void );
#line 59
static inline uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void );
#line 65
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void );
#line 65
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 59
static inline uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void );
#line 56
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void );
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void );





static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void );
#line 56
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void );
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );

static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle(void );




static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );

static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle(void );




static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );

static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle(void );




static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__toggle(void );
#line 85
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );

static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle(void );



static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__toggle(void );
#line 85
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );

static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle(void );



static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle(void );
#line 85
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );

static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void );



static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
# 73 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static void BlinkToRadioC__Timer0__startOneShot(uint32_t dt);
# 104 "/opt/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
static error_t BlinkToRadioC__AMControl__start(void );
# 80 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static error_t BlinkToRadioC__AMSend__send(am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 63 "/opt/tinyos-2.1.2/tos/interfaces/PacketTimeStamp.nc"
static BlinkToRadioC__PacketTimeStamp__size_type BlinkToRadioC__PacketTimeStamp__timestamp(
#line 52
message_t * msg);
# 67 "/opt/tinyos-2.1.2/tos/interfaces/Leds.nc"
static void BlinkToRadioC__Leds__led0Toggle(void );
#line 83
static void BlinkToRadioC__Leds__led1Toggle(void );
#line 100
static void BlinkToRadioC__Leds__led2Toggle(void );
# 36 "BlinkToRadioC.nc"
message_t *BlinkToRadioC__pktptr;
bool BlinkToRadioC__busy = FALSE;





uint32_t BlinkToRadioC__receiveTS;
uint32_t BlinkToRadioC__resendTS;
uint32_t BlinkToRadioC__received_counter;
#line 63
static inline void BlinkToRadioC__Boot__booted(void );




static inline void BlinkToRadioC__AMControl__startDone(error_t err);







static inline void BlinkToRadioC__AMControl__stopDone(error_t err);


static inline void BlinkToRadioC__Timer0__fired(void );






static void BlinkToRadioC__AMSend__sendDone(message_t *msg, error_t err);







static inline message_t *BlinkToRadioC__Receive__receive(message_t *msg, void *payload, uint8_t len);
# 41 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void );
# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void );
# 57 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void );
#line 47
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void );
#line 44
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 53 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
#line 65
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 114
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void );
# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 49 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void );
# 67 "/opt/tinyos-2.1.2/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC__0____nesc_unnamed4308 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 5, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) + 5, 



  TransformCounterC__0__OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
#line 133
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void );
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void );
# 77 "/opt/tinyos-2.1.2/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0____nesc_unnamed4309 {

  TransformAlarmC__0__MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type ) - 1 - 5, 
  TransformAlarmC__0__MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void );
#line 147
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt);
#line 162
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
#line 177
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void );
# 109 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void );
# 83 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void );
# 74 "/opt/tinyos-2.1.2/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4310 {
#line 74
  AlarmToTimerC__0__fired = 0U
};
#line 74
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired];
#line 55
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot);
#line 71
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );


static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
#line 93
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void );
# 136 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void );
#line 129
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(
# 48 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b395a6cd5d8);
#line 71
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4311 {
#line 71
  VirtualizeTimerC__0__updateFromTimer = 1U
};
#line 71
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 53
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4312 {

  VirtualizeTimerC__0__NUM_TIMERS = 1U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 59
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4313 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now);
#line 100
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
#line 139
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);
#line 159
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt);
#line 204
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 58 "/opt/tinyos-2.1.2/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 65 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static tradio_size CC2420XRadioP__RadioAlarm__getNow(void );
# 120 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
static uint8_t CC2420XRadioP__Ieee154PacketLayer__getDSN(message_t *msg);
#line 75
static bool CC2420XRadioP__Ieee154PacketLayer__isAckFrame(message_t *msg);
#line 156
static void CC2420XRadioP__Ieee154PacketLayer__setSrcAddr(message_t *msg, uint16_t addr);
#line 69
static void CC2420XRadioP__Ieee154PacketLayer__createDataFrame(message_t *msg);
#line 125
static void CC2420XRadioP__Ieee154PacketLayer__setDSN(message_t *msg, uint8_t dsn);
#line 88
static void CC2420XRadioP__Ieee154PacketLayer__createAckReply(message_t *data, message_t *ack);
#line 151
static uint16_t CC2420XRadioP__Ieee154PacketLayer__getSrcAddr(message_t *msg);
#line 63
static bool CC2420XRadioP__Ieee154PacketLayer__isDataFrame(message_t *msg);
#line 94
static bool CC2420XRadioP__Ieee154PacketLayer__verifyAckReply(message_t *data, message_t *ack);
#line 162
static bool CC2420XRadioP__Ieee154PacketLayer__requiresAckWait(message_t *msg);
#line 146
static void CC2420XRadioP__Ieee154PacketLayer__setDestAddr(message_t *msg, uint16_t addr);
#line 136
static void CC2420XRadioP__Ieee154PacketLayer__setDestPan(message_t *msg, uint16_t pan);
#line 168
static bool CC2420XRadioP__Ieee154PacketLayer__requiresAckReply(message_t *msg);
#line 141
static uint16_t CC2420XRadioP__Ieee154PacketLayer__getDestAddr(message_t *msg);
# 62 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline uint8_t CC2420XRadioP__CC2420XDriverConfig__headerLength(message_t *msg);




static inline uint8_t CC2420XRadioP__CC2420XDriverConfig__maxPayloadLength(void );




static inline uint8_t CC2420XRadioP__CC2420XDriverConfig__metadataLength(message_t *msg);




static inline uint8_t CC2420XRadioP__CC2420XDriverConfig__headerPreloadLength(void );





static inline bool CC2420XRadioP__CC2420XDriverConfig__requiresRssiCca(message_t *msg);






static inline bool CC2420XRadioP__SoftwareAckConfig__requiresAckWait(message_t *msg);




static inline bool CC2420XRadioP__SoftwareAckConfig__isAckPacket(message_t *msg);




static inline bool CC2420XRadioP__SoftwareAckConfig__verifyAckPacket(message_t *data, message_t *ack);









static inline bool CC2420XRadioP__SoftwareAckConfig__requiresAckReply(message_t *msg);




static inline void CC2420XRadioP__SoftwareAckConfig__createAckPacket(message_t *data, message_t *ack);








static inline uint16_t CC2420XRadioP__SoftwareAckConfig__getAckTimeout(void );




static inline void CC2420XRadioP__SoftwareAckConfig__reportChannelError(void );








static inline uint8_t CC2420XRadioP__UniqueConfig__getSequenceNumber(message_t *msg);




static inline void CC2420XRadioP__UniqueConfig__setSequenceNumber(message_t *msg, uint8_t dsn);




static inline am_addr_t CC2420XRadioP__UniqueConfig__getSender(message_t *msg);




static inline void CC2420XRadioP__UniqueConfig__reportChannelError(void );








static inline am_addr_t CC2420XRadioP__ActiveMessageConfig__destination(message_t *msg);




static inline void CC2420XRadioP__ActiveMessageConfig__setDestination(message_t *msg, am_addr_t addr);









static inline void CC2420XRadioP__ActiveMessageConfig__setSource(message_t *msg, am_addr_t addr);









static inline void CC2420XRadioP__ActiveMessageConfig__setGroup(message_t *msg, am_group_t grp);




static inline error_t CC2420XRadioP__ActiveMessageConfig__checkFrame(message_t *msg);
#line 229
static inline uint16_t CC2420XRadioP__RandomCollisionConfig__getMinimumBackoff(void );




static inline uint16_t CC2420XRadioP__RandomCollisionConfig__getInitialBackoff(message_t *msg);




static inline uint16_t CC2420XRadioP__RandomCollisionConfig__getCongestionBackoff(message_t *msg);






static inline uint16_t CC2420XRadioP__RandomCollisionConfig__getTransmitBarrier(message_t *msg);
#line 262
static inline void CC2420XRadioP__RadioAlarm__fired(void );
# 60 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__fired(
# 43 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
uint8_t arg_0x2b395a8ab220);
# 109 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__size_type /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__getNow(void );
#line 66
static void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__start(/*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__size_type dt);






static void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__stop(void );
# 59 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
static void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Tasklet__schedule(void );
# 55 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
uint8_t /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__state;
enum /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0____nesc_unnamed4314 {

  RadioAlarmP__0__STATE_READY = 0, 
  RadioAlarmP__0__STATE_WAIT = 1, 
  RadioAlarmP__0__STATE_FIRED = 2
};

uint8_t /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__alarm;

static inline void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__fired(void );










static __inline tradio_size /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__getNow(uint8_t id);




static inline void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Tasklet__run(void );








static inline void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__default__fired(uint8_t id);



static __inline bool /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__isFree(uint8_t id);




static inline void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__wait(uint8_t id, tradio_size timeout);








static inline void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__cancel(uint8_t id);
# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
static void TaskletC__Tasklet__run(void );
# 72 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/TaskletC.nc"
uint8_t TaskletC__state;

static void TaskletC__doit(void );
#line 94
static __inline void TaskletC__Tasklet__suspend(void );




static void TaskletC__Tasklet__resume(void );
#line 112
static void TaskletC__Tasklet__schedule(void );
# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
static error_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubSend__send(message_t *msg);
# 59 "/opt/tinyos-2.1.2/tos/interfaces/SendNotifier.nc"
static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SendNotifier__aboutToSend(
# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
am_id_t arg_0x2b395a917df8, 
# 59 "/opt/tinyos-2.1.2/tos/interfaces/SendNotifier.nc"
am_addr_t dest, 
#line 57
message_t * msg);
# 110 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMSend__sendDone(
# 45 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
am_id_t arg_0x2b395a91a5f0, 
# 103 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Snoop__receive(
# 47 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
am_id_t arg_0x2b395a9172f0, 
# 71 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 63 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageConfig.nc"
static error_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Config__checkFrame(message_t *msg);
#line 40
static am_addr_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Config__destination(message_t *msg);








static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Config__setSource(message_t *msg, am_addr_t addr);
#line 43
static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Config__setDestination(message_t *msg, am_addr_t addr);
#line 55
static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Config__setGroup(message_t *msg, am_group_t grp);
# 50 "/opt/tinyos-2.1.2/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__ActiveMessageAddress__amAddress(void );




static am_group_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__ActiveMessageAddress__amGroup(void );
# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
static uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubPacket__payloadLength(message_t *msg);









static uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubPacket__maxPayloadLength(void );
#line 54
static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 43
static uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubPacket__headerLength(message_t *msg);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Receive__receive(
# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
am_id_t arg_0x2b395a919780, 
# 71 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
#line 78
static 
#line 74
message_t * 



/*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__ReceiveDefault__receive(
# 51 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
am_id_t arg_0x2b395a916a38, 
# 71 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
#line 78
static 
#line 74
message_t * 



/*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SnoopDefault__receive(
# 52 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
am_id_t arg_0x2b395a9156b8, 
# 71 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 67 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
static inline activemessage_header_t */*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__getHeader(message_t *msg);




static inline void */*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__getPayload(message_t *msg);






static error_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMSend__send(am_id_t id, am_addr_t addr, message_t *msg, uint8_t len);
#line 98
static __inline void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubSend__sendDone(message_t *msg, error_t error);
#line 122
static inline void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SendNotifier__default__aboutToSend(am_id_t id, am_addr_t addr, message_t *msg);





static inline message_t */*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubReceive__receive(message_t *msg);
#line 141
static inline message_t */*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Receive__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);




static inline message_t */*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__ReceiveDefault__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);




static inline message_t */*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Snoop__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);




static inline message_t */*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SnoopDefault__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);






static __inline am_addr_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__address(void );




static __inline am_group_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__localGroup(void );




static __inline bool /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__isForMe(message_t *msg);





static __inline am_addr_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__destination(message_t *msg);




static __inline void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__setDestination(message_t *msg, am_addr_t addr);









static __inline void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__setSource(message_t *msg, am_addr_t addr);




static __inline am_id_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__type(message_t *msg);




static __inline void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__setType(message_t *msg, am_id_t type);









static __inline void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__setGroup(message_t *msg, am_group_t grp);










static inline uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__RadioPacket__headerLength(message_t *msg);




static inline uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__RadioPacket__payloadLength(message_t *msg);




static inline void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);




static inline uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__RadioPacket__maxPayloadLength(void );
#line 262
static inline uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Packet__payloadLength(message_t *msg);




static inline void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Packet__setPayloadLength(message_t *msg, uint8_t len);




static inline uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Packet__maxPayloadLength(void );
# 62 "/opt/tinyos-2.1.2/tos/system/ActiveMessageAddressC.nc"
am_addr_t ActiveMessageAddressC__addr = TOS_AM_ADDRESS;


am_group_t ActiveMessageAddressC__group = TOS_AM_GROUP;






static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );
#line 93
static am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
#line 106
static am_addr_t ActiveMessageAddressC__amAddress(void );
# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
static error_t /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__SubSend__send(message_t *msg);
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__release(void );
#line 97
static error_t /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__immediateRequest(void );
#line 88
static error_t /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__request(void );
# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
static void /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__BareSend__sendDone(message_t *msg, error_t error);
# 51 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/AutoResourceAcquireLayerC.nc"
message_t */*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__pending;

static inline error_t /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__BareSend__send(message_t *msg);
#line 68
static inline void /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__granted(void );









static inline void /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__SubSend__sendDone(message_t *msg, error_t result);
# 49 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
enum /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0____nesc_unnamed4315 {
#line 49
  FcfsResourceQueueC__0__NO_ENTRY = 0xFF
};
uint8_t /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__resQ[1U];
uint8_t /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qHead = /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
uint8_t /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qTail = /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

static inline error_t /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__Init__init(void );




static inline bool /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );



static inline bool /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
#line 82
static inline error_t /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
# 53 "/opt/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__requested(
# 52 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b395a9fe1a0);
# 61 "/opt/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__immediateRequested(
# 52 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b395a9fe1a0);
# 65 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__unconfigure(
# 56 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b395a9fd6e0);
# 59 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(
# 56 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b395a9fd6e0);
# 79 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
static error_t /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__enqueue(resource_client_id_t id);
#line 53
static bool /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__isEmpty(void );
#line 70
static resource_client_id_t /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__dequeue(void );
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__granted(
# 51 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b395a9be020);
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__postTask(void );
# 69 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
enum /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0____nesc_unnamed4316 {
#line 69
  SimpleArbiterP__0__grantedTask = 2U
};
#line 69
typedef int /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0____nesc_sillytask_grantedTask[/*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask];
#line 62
enum /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0____nesc_unnamed4317 {
#line 62
  SimpleArbiterP__0__RES_IDLE = 0, SimpleArbiterP__0__RES_GRANTING = 1, SimpleArbiterP__0__RES_BUSY = 2
};
#line 63
enum /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0____nesc_unnamed4318 {
#line 63
  SimpleArbiterP__0__NO_RES = 0xFF
};
uint8_t /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__state = /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_IDLE;
uint8_t /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId = /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__NO_RES;
uint8_t /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__reqResId;



static inline error_t /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__request(uint8_t id);
#line 84
static inline error_t /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__immediateRequest(uint8_t id);
#line 97
static error_t /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__release(uint8_t id);
#line 155
static inline void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__runTask(void );









static inline void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__default__granted(uint8_t id);

static inline void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(uint8_t id);

static inline void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id);

static inline void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(uint8_t id);

static inline void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id);
# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
static uint8_t /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__RadioPacket__payloadLength(message_t *msg);
#line 43
static uint8_t /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__RadioPacket__headerLength(message_t *msg);
# 185 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
static bool /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Ieee154PacketLayer__isForMe(message_t *msg);
# 97 "/opt/tinyos-2.1.2/tos/interfaces/Ieee154Send.nc"
static void /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Ieee154Send__sendDone(message_t *msg, error_t error);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Ieee154Receive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 56 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154MessageLayerC.nc"
static inline void */*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__getPayload(message_t *msg);
#line 68
static inline uint8_t /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Packet__payloadLength(message_t *msg);
#line 127
static inline void /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__SubSend__sendDone(message_t *msg, error_t error);




static inline void /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Ieee154Send__default__sendDone(message_t *msg, error_t error);









static inline message_t */*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__SubReceive__receive(message_t *msg);








static inline message_t */*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Ieee154Receive__default__receive(message_t *msg, void *payload, uint8_t len);
# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
static error_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubSend__send(message_t *msg);
# 42 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareReceive.nc"
static message_t */*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosReceive__receive(message_t *msg);
# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
static uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubPacket__payloadLength(message_t *msg);









static uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubPacket__maxPayloadLength(void );
#line 54
static void /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 43
static uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubPacket__headerLength(message_t *msg);
# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
static void /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosSend__sendDone(message_t *msg, error_t error);
#line 54
static void /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__Ieee154Send__sendDone(message_t *msg, error_t error);
# 42 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareReceive.nc"
static message_t */*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__Ieee154Receive__receive(message_t *msg);
# 91 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TinyosNetworkLayerC.nc"
static inline uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__Ieee154Packet__headerLength(message_t *msg);




static inline uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__Ieee154Packet__payloadLength(message_t *msg);
#line 127
static inline network_header_t */*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__getHeader(message_t *msg);




static error_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosSend__send(message_t *msg);
#line 145
enum /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0____nesc_unnamed4319 {


  TinyosNetworkLayerC__0__PAYLOAD_OFFSET = sizeof(network_header_t )
};




static inline uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosPacket__headerLength(message_t *msg);




static inline uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosPacket__payloadLength(message_t *msg);




static inline void /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosPacket__setPayloadLength(message_t *msg, uint8_t length);




static inline uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosPacket__maxPayloadLength(void );
#line 214
static inline void /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubSend__sendDone(message_t *msg, error_t result);







static inline message_t */*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubReceive__receive(message_t *msg);
# 50 "/opt/tinyos-2.1.2/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__ActiveMessageAddress__amAddress(void );




static am_group_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__ActiveMessageAddress__amGroup(void );
# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
static uint8_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__payloadLength(message_t *msg);









static uint8_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__maxPayloadLength(void );
#line 54
static void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 43
static uint8_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__headerLength(message_t *msg);
# 57 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
enum /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0____nesc_unnamed4320 {

  Ieee154PacketLayerP__0__IEEE154_DATA_FRAME_MASK = (((IEEE154_TYPE_MASK << IEEE154_FCF_FRAME_TYPE)
   | (1 << IEEE154_FCF_INTRAPAN))
   | (IEEE154_ADDR_MASK << IEEE154_FCF_DEST_ADDR_MODE))
   | (IEEE154_ADDR_MASK << IEEE154_FCF_SRC_ADDR_MODE), 

  Ieee154PacketLayerP__0__IEEE154_DATA_FRAME_VALUE = (((IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE)
   | (1 << IEEE154_FCF_INTRAPAN))
   | (IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE))
   | (IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE), 

  Ieee154PacketLayerP__0__IEEE154_DATA_FRAME_PRESERVE = (1 << IEEE154_FCF_ACK_REQ)
   | (1 << IEEE154_FCF_FRAME_PENDING), 

  Ieee154PacketLayerP__0__IEEE154_ACK_FRAME_LENGTH = 3, 
  Ieee154PacketLayerP__0__IEEE154_ACK_FRAME_MASK = IEEE154_TYPE_MASK << IEEE154_FCF_FRAME_TYPE, 
  Ieee154PacketLayerP__0__IEEE154_ACK_FRAME_VALUE = IEEE154_TYPE_ACK << IEEE154_FCF_FRAME_TYPE
};

static inline ieee154_simple_header_t */*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(message_t *msg);
#line 92
static bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isDataFrame(message_t *msg);




static inline void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__createDataFrame(message_t *msg);






static bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isAckFrame(message_t *msg);










static inline void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__createAckReply(message_t *data, message_t *ack);








static inline bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__verifyAckReply(message_t *data, message_t *ack);







static bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getAckRequired(message_t *msg);
#line 158
static inline uint8_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDSN(message_t *msg);




static inline void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDSN(message_t *msg, uint8_t dsn);




static inline uint16_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDestPan(message_t *msg);




static inline void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDestPan(message_t *msg, uint16_t pan);




static uint16_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDestAddr(message_t *msg);




static inline void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDestAddr(message_t *msg, uint16_t addr);




static inline uint16_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getSrcAddr(message_t *msg);




static inline void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setSrcAddr(message_t *msg, uint16_t addr);




static inline bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__requiresAckWait(message_t *msg);






static bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__requiresAckReply(message_t *msg);






static inline ieee154_saddr_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__localAddr(void );




static inline ieee154_panid_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__localPan(void );




static inline bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isForMe(message_t *msg);
#line 282
static uint8_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__RadioPacket__headerLength(message_t *msg);




static inline uint8_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__RadioPacket__payloadLength(message_t *msg);




static void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);




static inline uint8_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__RadioPacket__maxPayloadLength(void );
# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
static error_t /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__SubSend__send(message_t *msg);
# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/NeighborhoodFlag.nc"
static bool /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__NeighborhoodFlag__get(uint8_t idx);




static void /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__NeighborhoodFlag__set(uint8_t idx);
# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
static void /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__Send__sendDone(message_t *msg, error_t error);
# 71 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/Neighborhood.nc"
static uint8_t /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__Neighborhood__insertNode(am_addr_t id);
# 53 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
static message_t */*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__RadioReceive__receive(message_t *msg);
#line 46
static bool /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__RadioReceive__header(message_t *msg);
# 52 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/UniqueConfig.nc"
static void /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__UniqueConfig__setSequenceNumber(message_t *msg, uint8_t number);





static void /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__UniqueConfig__reportChannelError(void );
#line 42
static uint8_t /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__UniqueConfig__getSequenceNumber(message_t *msg);




static am_addr_t /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__UniqueConfig__getSender(message_t *msg);
# 61 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/UniqueLayerP.nc"
uint8_t /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__sequenceNumber;

static inline error_t /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__Init__init(void );





static inline error_t /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__Send__send(message_t *msg);










static inline void /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__SubSend__sendDone(message_t *msg, error_t error);




static inline bool /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__SubReceive__header(message_t *msg);





uint8_t /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__receivedNumbers[5];

static inline message_t */*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__SubReceive__receive(message_t *msg);
#line 116
static inline void /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__Neighborhood__evicted(uint8_t idx);
# 80 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/Neighborhood.nc"
static void NeighborhoodP__Neighborhood__evicted(uint8_t idx);
# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/NeighborhoodP.nc"
am_addr_t NeighborhoodP__nodes[5];
uint8_t NeighborhoodP__ages[5];
uint8_t NeighborhoodP__flags[5];
uint8_t NeighborhoodP__time;
uint8_t NeighborhoodP__last;

static inline error_t NeighborhoodP__Init__init(void );
#line 94
static inline uint8_t NeighborhoodP__Neighborhood__insertNode(am_addr_t node);
#line 158
static __inline bool NeighborhoodP__NeighborhoodFlag__get(uint8_t bit, uint8_t idx);




static __inline void NeighborhoodP__NeighborhoodFlag__set(uint8_t bit, uint8_t idx);
# 113 "/opt/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__SplitControl__startDone(error_t error);
#line 138
static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__SplitControl__stopDone(error_t error);
# 56 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioState.nc"
static error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioState__turnOn(void );
# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Send__sendDone(message_t *msg, error_t error);
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__sendTask__postTask(void );
#line 67
static error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__stateDoneTask__postTask(void );
# 42 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareReceive.nc"
static message_t */*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Receive__receive(message_t *msg);
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__deliverTask__postTask(void );
# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioSend__send(message_t *msg);
# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioChannel.nc"
static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioChannel__setChannelDone(void );
# 72 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Tasklet__suspend(void );






static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Tasklet__resume(void );
# 144 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MessageBufferLayerP.nc"
enum /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0____nesc_unnamed4321 {
#line 144
  MessageBufferLayerP__0__stateDoneTask = 3U
};
#line 144
typedef int /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0____nesc_sillytask_stateDoneTask[/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__stateDoneTask];
#line 189
enum /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0____nesc_unnamed4322 {
#line 189
  MessageBufferLayerP__0__sendTask = 4U
};
#line 189
typedef int /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0____nesc_sillytask_sendTask[/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__sendTask];
#line 322
enum /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0____nesc_unnamed4323 {
#line 322
  MessageBufferLayerP__0__deliverTask = 5U
};
#line 322
typedef int /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0____nesc_sillytask_deliverTask[/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__deliverTask];
#line 63
uint8_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__state;
enum /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0____nesc_unnamed4324 {

  MessageBufferLayerP__0__STATE_READY = 0, 
  MessageBufferLayerP__0__STATE_TX_PENDING = 1, 
  MessageBufferLayerP__0__STATE_TX_RETRY = 2, 
  MessageBufferLayerP__0__STATE_TX_SEND = 3, 
  MessageBufferLayerP__0__STATE_TX_DONE = 4, 
  MessageBufferLayerP__0__STATE_TURN_ON = 5, 
  MessageBufferLayerP__0__STATE_TURN_OFF = 6, 
  MessageBufferLayerP__0__STATE_CHANNEL = 7
};

static error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__SplitControl__start(void );
#line 144
static inline void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__stateDoneTask__runTask(void );
#line 163
static inline void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioState__done(void );
#line 176
static inline void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioChannel__default__setChannelDone(void );





message_t */*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__txMsg;
error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__txError;
uint8_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__retries;


enum /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0____nesc_unnamed4325 {
#line 187
  MessageBufferLayerP__0__MAX_RETRIES = 5
};
static inline void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__sendTask__runTask(void );
#line 217
static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioSend__sendDone(error_t error);
#line 230
static inline error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Send__send(message_t *msg);
#line 252
static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioSend__ready(void );








static inline void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Tasklet__run(void );
#line 291
enum /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0____nesc_unnamed4326 {

  MessageBufferLayerP__0__RECEIVE_QUEUE_SIZE = 3
};

message_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueueData[/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RECEIVE_QUEUE_SIZE];
message_t */*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueue[/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RECEIVE_QUEUE_SIZE];

uint8_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueueHead;
uint8_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueueSize;

static inline error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__SoftwareInit__init(void );









static inline bool /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioReceive__header(message_t *msg);









static inline void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__deliverTask__runTask(void );
#line 351
static inline message_t */*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioReceive__receive(message_t *msg);
# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static error_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubSend__send(message_t *msg);
# 50 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__wait(tradio_size timeout);
#line 45
static bool /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__isFree(void );
#line 65
static tradio_size /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__getNow(void );
# 52 "/opt/tinyos-2.1.2/tos/interfaces/Random.nc"
static uint16_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__Random__rand16(void );
# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/RandomCollisionConfig.nc"
static uint16_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__Config__getCongestionBackoff(message_t *msg);
#line 40
static uint16_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__Config__getInitialBackoff(message_t *msg);










static uint16_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__Config__getMinimumBackoff(void );





static uint16_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__Config__getTransmitBarrier(message_t *msg);
# 53 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
static message_t */*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioReceive__receive(message_t *msg);
#line 46
static bool /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioReceive__header(message_t *msg);
# 63 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static void /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioSend__ready(void );
#line 56
static void /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioSend__sendDone(error_t error);
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__calcNextRandom__postTask(void );
# 78 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/RandomCollisionLayerP.nc"
enum /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0____nesc_unnamed4327 {
#line 78
  RandomCollisionLayerP__0__calcNextRandom = 6U
};
#line 78
typedef int /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0____nesc_sillytask_calcNextRandom[/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__calcNextRandom];
#line 57
uint8_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__state;
enum /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0____nesc_unnamed4328 {

  RandomCollisionLayerP__0__STATE_READY = 0, 
  RandomCollisionLayerP__0__STATE_TX_PENDING_FIRST = 1, 
  RandomCollisionLayerP__0__STATE_TX_PENDING_SECOND = 2, 
  RandomCollisionLayerP__0__STATE_TX_SENDING = 3, 

  RandomCollisionLayerP__0__STATE_BARRIER = 0x80
};

message_t */*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__txMsg;
uint16_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__txBarrier;

static inline void /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubSend__ready(void );





uint16_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__nextRandom;
static inline void /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__calcNextRandom__runTask(void );





static uint16_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__getBackoff(uint16_t maxBackoff);
#line 98
static inline error_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioSend__send(message_t *msg);
#line 110
static inline void /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__fired(void );
#line 155
static inline void /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubSend__sendDone(error_t error);







static inline bool /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubReceive__header(message_t *msg);




static inline message_t */*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubReceive__receive(message_t *msg);
# 52 "/opt/tinyos-2.1.2/tos/system/RandomMlcgC.nc"
uint32_t RandomMlcgC__seed;


static inline error_t RandomMlcgC__Init__init(void );
#line 69
static uint32_t RandomMlcgC__Random__rand32(void );
#line 89
static inline uint16_t RandomMlcgC__Random__rand16(void );
# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static error_t /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubSend__send(message_t *msg);
# 50 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioAlarm__wait(tradio_size timeout);




static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioAlarm__cancel(void );
#line 45
static bool /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioAlarm__isFree(void );
# 55 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__AckReceivedFlag__clear(message_t *msg);
#line 50
static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__AckReceivedFlag__set(message_t *msg);
# 86 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__reportChannelError(void );
#line 80
static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__createAckPacket(message_t *data, message_t *ack);
#line 55
static bool /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__requiresAckWait(message_t *msg);






static bool /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__isAckPacket(message_t *msg);






static bool /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__verifyAckPacket(message_t *data, message_t *ack);
#line 43
static uint16_t /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__getAckTimeout(void );
#line 75
static bool /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__requiresAckReply(message_t *msg);
# 53 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
static message_t */*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioReceive__receive(message_t *msg);
#line 46
static bool /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioReceive__header(message_t *msg);
# 63 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioSend__ready(void );
#line 56
static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioSend__sendDone(error_t error);
# 60 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckLayerC.nc"
uint8_t /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__state;
enum /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0____nesc_unnamed4329 {

  SoftwareAckLayerC__0__STATE_READY = 0, 
  SoftwareAckLayerC__0__STATE_DATA_SEND = 1, 
  SoftwareAckLayerC__0__STATE_ACK_WAIT = 2, 
  SoftwareAckLayerC__0__STATE_ACK_SEND = 3
};

message_t */*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__txMsg;
message_t /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__ackMsg;

static inline void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubSend__ready(void );





static inline error_t /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioSend__send(message_t *msg);
#line 97
static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubSend__sendDone(error_t error);
#line 124
static inline void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioAlarm__fired(void );









static inline bool /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubReceive__header(message_t *msg);
#line 147
static inline message_t */*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubReceive__receive(message_t *msg);
# 50 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
static void /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__TimeStampFlag__set(message_t *msg);
# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
static uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__SubPacket__payloadLength(message_t *msg);









static uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__SubPacket__maxPayloadLength(void );
#line 54
static void /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__SubPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 43
static uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__SubPacket__headerLength(message_t *msg);
#line 65
static uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__SubPacket__metadataLength(message_t *msg);
# 60 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TimeStampingLayerP.nc"
static inline timestamp_metadata_t */*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__getMeta(message_t *msg);
#line 72
static uint32_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__PacketTimeStampRadio__timestamp(message_t *msg);









static inline void /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__PacketTimeStampRadio__set(message_t *msg, uint32_t value);
#line 116
static inline uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__headerLength(message_t *msg);




static inline uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__payloadLength(message_t *msg);




static inline void /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);




static inline uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__maxPayloadLength(void );




static inline uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__metadataLength(message_t *msg);
# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
static uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__SubPacket__payloadLength(message_t *msg);









static uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__SubPacket__maxPayloadLength(void );
#line 54
static void /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__SubPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 43
static uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__SubPacket__headerLength(message_t *msg);
#line 65
static uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__SubPacket__metadataLength(message_t *msg);
# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MetadataFlagsLayerC.nc"
static inline flags_metadata_t */*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__getMeta(message_t *msg);






static bool /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__PacketFlag__get(uint8_t bit, message_t *msg);




static void /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__PacketFlag__set(uint8_t bit, message_t *msg);






static inline void /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__PacketFlag__clear(uint8_t bit, message_t *msg);
#line 90
static inline uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__headerLength(message_t *msg);




static inline uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__payloadLength(message_t *msg);




static inline void /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);




static inline uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__maxPayloadLength(void );




static inline uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__metadataLength(message_t *msg);
# 44 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void CC2420XDriverLayerP__FIFO__makeInput(void );
#line 43
static bool CC2420XDriverLayerP__FIFO__get(void );
# 66 "/opt/tinyos-2.1.2/tos/lib/timer/BusyWait.nc"
static void CC2420XDriverLayerP__BusyWait__wait(CC2420XDriverLayerP__BusyWait__size_type dt);
# 50 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
static void CC2420XDriverLayerP__RSSIFlag__set(message_t *msg);
# 44 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void CC2420XDriverLayerP__FIFOP__makeInput(void );
#line 43
static bool CC2420XDriverLayerP__FIFOP__get(void );
# 78 "/opt/tinyos-2.1.2/tos/interfaces/PacketTimeStamp.nc"
static void CC2420XDriverLayerP__PacketTimeStamp__set(
#line 73
message_t * msg, 




CC2420XDriverLayerP__PacketTimeStamp__size_type value);
# 46 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void CC2420XDriverLayerP__RSTN__makeOutput(void );
#line 40
static void CC2420XDriverLayerP__RSTN__set(void );
static void CC2420XDriverLayerP__RSTN__clr(void );
# 50 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void CC2420XDriverLayerP__RadioAlarm__wait(tradio_size timeout);
#line 45
static bool CC2420XDriverLayerP__RadioAlarm__isFree(void );
#line 65
static tradio_size CC2420XDriverLayerP__RadioAlarm__getNow(void );
# 69 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioState.nc"
static void CC2420XDriverLayerP__RadioState__done(void );
# 61 "/opt/tinyos-2.1.2/tos/lib/timer/LocalTime.nc"
static uint32_t CC2420XDriverLayerP__LocalTime__get(void );
# 61 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420XDriverLayerP__FifopInterrupt__disable(void );
# 35 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverConfig.nc"
static uint8_t CC2420XDriverLayerP__Config__maxPayloadLength(void );
#line 29
static uint8_t CC2420XDriverLayerP__Config__headerLength(message_t *msg);
#line 41
static uint8_t CC2420XDriverLayerP__Config__metadataLength(message_t *msg);






static uint8_t CC2420XDriverLayerP__Config__headerPreloadLength(void );





static bool CC2420XDriverLayerP__Config__requiresRssiCca(message_t *msg);
# 53 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
static message_t *CC2420XDriverLayerP__RadioReceive__receive(message_t *msg);
#line 46
static bool CC2420XDriverLayerP__RadioReceive__header(message_t *msg);
# 46 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void CC2420XDriverLayerP__CSN__makeOutput(void );
#line 40
static void CC2420XDriverLayerP__CSN__set(void );
static void CC2420XDriverLayerP__CSN__clr(void );




static void CC2420XDriverLayerP__VREN__makeOutput(void );
#line 40
static void CC2420XDriverLayerP__VREN__set(void );
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t CC2420XDriverLayerP__releaseSpi__postTask(void );
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t CC2420XDriverLayerP__SpiResource__release(void );
#line 97
static error_t CC2420XDriverLayerP__SpiResource__immediateRequest(void );
#line 88
static error_t CC2420XDriverLayerP__SpiResource__request(void );
#line 128
static bool CC2420XDriverLayerP__SpiResource__isOwner(void );
# 44 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void CC2420XDriverLayerP__CCA__makeInput(void );
#line 43
static bool CC2420XDriverLayerP__CCA__get(void );
static void CC2420XDriverLayerP__SFD__makeInput(void );
#line 43
static bool CC2420XDriverLayerP__SFD__get(void );
# 63 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static void CC2420XDriverLayerP__RadioSend__ready(void );
#line 56
static void CC2420XDriverLayerP__RadioSend__sendDone(error_t error);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/FastSpiByte.nc"
static void CC2420XDriverLayerP__FastSpiByte__splitWrite(uint8_t data);
#line 74
static uint8_t CC2420XDriverLayerP__FastSpiByte__splitReadWrite(uint8_t data);
#line 68
static uint8_t CC2420XDriverLayerP__FastSpiByte__splitRead(void );
# 54 "/opt/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
static error_t CC2420XDriverLayerP__SfdCapture__captureFallingEdge(void );
#line 66
static void CC2420XDriverLayerP__SfdCapture__disable(void );
#line 53
static error_t CC2420XDriverLayerP__SfdCapture__captureRisingEdge(void );
# 40 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
static bool CC2420XDriverLayerP__TimeSyncFlag__get(message_t *msg);
#line 55
static void CC2420XDriverLayerP__TransmitPowerFlag__clear(message_t *msg);
#line 40
static bool CC2420XDriverLayerP__TransmitPowerFlag__get(message_t *msg);
# 59 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
static void CC2420XDriverLayerP__Tasklet__schedule(void );
# 1102 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
enum CC2420XDriverLayerP____nesc_unnamed4330 {
#line 1102
  CC2420XDriverLayerP__releaseSpi = 7U
};
#line 1102
typedef int CC2420XDriverLayerP____nesc_sillytask_releaseSpi[CC2420XDriverLayerP__releaseSpi];
#line 85
static inline cc2420x_header_t *CC2420XDriverLayerP__getHeader(message_t *msg);




static inline void *CC2420XDriverLayerP__getPayload(message_t *msg);




static cc2420x_metadata_t *CC2420XDriverLayerP__getMeta(message_t *msg);






enum CC2420XDriverLayerP____nesc_unnamed4331 {

  CC2420XDriverLayerP__STATE_VR_ON = 0, 
  CC2420XDriverLayerP__STATE_PD = 1, 
  CC2420XDriverLayerP__STATE_PD_2_IDLE = 2, 
  CC2420XDriverLayerP__STATE_IDLE = 3, 
  CC2420XDriverLayerP__STATE_IDLE_2_RX_ON = 4, 
  CC2420XDriverLayerP__STATE_RX_ON = 5, 
  CC2420XDriverLayerP__STATE_BUSY_TX_2_RX_ON = 6, 
  CC2420XDriverLayerP__STATE_IDLE_2_TX_ON = 7, 
  CC2420XDriverLayerP__STATE_TX_ON = 8, 
  CC2420XDriverLayerP__STATE_RX_DOWNLOAD = 9
};
uint8_t CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_VR_ON;

enum CC2420XDriverLayerP____nesc_unnamed4332 {

  CC2420XDriverLayerP__CMD_NONE = 0, 
  CC2420XDriverLayerP__CMD_TURNOFF = 1, 
  CC2420XDriverLayerP__CMD_STANDBY = 2, 
  CC2420XDriverLayerP__CMD_TURNON = 3, 
  CC2420XDriverLayerP__CMD_TRANSMIT = 4, 
  CC2420XDriverLayerP__CMD_RECEIVE = 5, 
  CC2420XDriverLayerP__CMD_CCA = 6, 
  CC2420XDriverLayerP__CMD_CHANNEL = 7, 
  CC2420XDriverLayerP__CMD_SIGNAL_DONE = 8, 
  CC2420XDriverLayerP__CMD_DOWNLOAD = 9
};
uint8_t CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_NONE;

bool CC2420XDriverLayerP__radioIrq = 0;

uint8_t CC2420XDriverLayerP__txPower;
uint8_t CC2420XDriverLayerP__channel;

message_t *CC2420XDriverLayerP__rxMsg;



message_t CC2420XDriverLayerP__rxMsgBuffer;

uint16_t CC2420XDriverLayerP__capturedTime;

static __inline cc2420X_status_t CC2420XDriverLayerP__getStatus(void );


static inline void CC2420XDriverLayerP__RadioAlarm__fired(void );
#line 189
static __inline cc2420X_status_t CC2420XDriverLayerP__strobe(uint8_t reg);
#line 207
static __inline cc2420X_status_t CC2420XDriverLayerP__getStatus(void );



static __inline cc2420X_status_t CC2420XDriverLayerP__writeRegister(uint8_t reg, uint16_t value);
#line 230
static __inline cc2420X_status_t CC2420XDriverLayerP__writeTxFifo(uint8_t *data, uint8_t length);
#line 249
static __inline uint8_t CC2420XDriverLayerP__waitForRxFifoNoTimeout(void );






static __inline uint8_t CC2420XDriverLayerP__waitForRxFifo(void );









static __inline cc2420X_status_t CC2420XDriverLayerP__readLengthFromRxFifo(uint8_t *lengthPtr);
#line 294
static __inline void CC2420XDriverLayerP__readPayloadFromRxFifo(uint8_t *data, uint8_t length);
#line 309
static __inline void CC2420XDriverLayerP__readRssiFromRxFifo(uint8_t *rssiPtr);









static __inline void CC2420XDriverLayerP__readCrcOkAndLqiFromRxFifo(uint8_t *crcOkAndLqiPtr);
#line 339
static inline error_t CC2420XDriverLayerP__SoftwareInit__init(void );
#line 373
static __inline void CC2420XDriverLayerP__resetRadio(void );
#line 397
static inline void CC2420XDriverLayerP__initRadio(void );










static inline void CC2420XDriverLayerP__SpiResource__granted(void );
#line 423
static bool CC2420XDriverLayerP__isSpiAcquired(void );
#line 463
static __inline void CC2420XDriverLayerP__setChannel(void );









static __inline void CC2420XDriverLayerP__changeChannel(void );
#line 493
static __inline void CC2420XDriverLayerP__changeState(void );
#line 585
static inline error_t CC2420XDriverLayerP__RadioState__turnOn(void );
#line 611
static error_t CC2420XDriverLayerP__RadioSend__send(message_t *msg);
#line 776
static __inline void CC2420XDriverLayerP__recover(void );
#line 819
static __inline void CC2420XDriverLayerP__downloadMessage(void );
#line 994
static inline void CC2420XDriverLayerP__SfdCapture__captured(uint16_t time);
#line 1030
static inline void CC2420XDriverLayerP__FifopInterrupt__fired(void );




static __inline void CC2420XDriverLayerP__serviceRadio(void );
#line 1102
static inline void CC2420XDriverLayerP__releaseSpi__runTask(void );




static inline void CC2420XDriverLayerP__Tasklet__run(void );
#line 1181
static inline uint8_t CC2420XDriverLayerP__RadioPacket__headerLength(message_t *msg);




static uint8_t CC2420XDriverLayerP__RadioPacket__payloadLength(message_t *msg);




static void CC2420XDriverLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);








static inline uint8_t CC2420XDriverLayerP__RadioPacket__maxPayloadLength(void );






static inline uint8_t CC2420XDriverLayerP__RadioPacket__metadataLength(message_t *msg);
#line 1219
static inline bool CC2420XDriverLayerP__PacketTransmitPower__isSet(message_t *msg);




static inline uint8_t CC2420XDriverLayerP__PacketTransmitPower__get(message_t *msg);
#line 1257
static inline void CC2420XDriverLayerP__PacketRSSI__set(message_t *msg, uint8_t value);










static inline bool CC2420XDriverLayerP__PacketTimeSyncOffset__isSet(message_t *msg);




static inline uint8_t CC2420XDriverLayerP__PacketTimeSyncOffset__get(message_t *msg);
#line 1307
static inline void CC2420XDriverLayerP__PacketLinkQuality__set(message_t *msg, uint8_t value);
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get(void );
# 58 "/opt/tinyos-2.1.2/tos/lib/timer/BusyWaitCounterC.nc"
enum /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0____nesc_unnamed4333 {

  BusyWaitCounterC__0__HALF_MAX_SIZE_TYPE = (/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type )1 << (8 * sizeof(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type ) - 1)
};

static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type dt);
#line 83
static inline void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow(void );
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__get(void );
# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__overflow(void );
# 49 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get(void );
#line 64
static inline void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void );
# 82 "/opt/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(
# 79 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af87ae8, 
# 75 "/opt/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(
# 82 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af85b40);
# 97 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(bool reset);
#line 177
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr(void );









static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isTxIntrPending(void );
#line 224
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(uint8_t data);
#line 168
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx(void );
#line 192
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending(void );
#line 158
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi(void );
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(
# 81 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af86898);
# 97 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(
# 81 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af86898);
# 88 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(
# 81 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af86898);
# 128 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(
# 81 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af86898);
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(
# 75 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2b395af389f8);
#line 102
enum /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_unnamed4334 {
#line 102
  Msp430SpiNoDmaP__0__signalDone_task = 8U
};
#line 102
typedef int /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_sillytask_signalDone_task[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task];
#line 91
enum /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_unnamed4335 {
  Msp430SpiNoDmaP__0__SPI_ATOMIC_SIZE = 2
};

uint16_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len;
uint8_t * /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf;
uint8_t * /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf;
uint16_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos;
uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_client;

static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void );


static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(uint8_t id);



static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(uint8_t id);



static inline bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(uint8_t id);



static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(uint8_t id);





static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(uint8_t id);
#line 146
static __inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitWrite(uint8_t data);



static __inline uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitRead(void );




static __inline uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitReadWrite(uint8_t data);
#line 172
static inline bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(uint8_t id);
static inline msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(uint8_t id);

static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp(void );
#line 227
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void );



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data);
#line 244
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void );




static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void );

static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error);
# 99 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__UCLK__selectIOFunc(void );
#line 92
static void HplMsp430Usart0P__UCLK__selectModuleFunc(void );
# 54 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data);
#line 49
static void HplMsp430Usart0P__Interrupts__txDone(void );
# 99 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__URXD__selectIOFunc(void );
#line 99
static void HplMsp430Usart0P__UTXD__selectIOFunc(void );
# 7 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430Usart0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430Usart0P__HplI2C__isI2C(void );
# 99 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SOMI__selectIOFunc(void );
#line 92
static void HplMsp430Usart0P__SOMI__selectModuleFunc(void );
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void HplMsp430Usart0P__I2CInterrupts__fired(void );
# 99 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SIMO__selectIOFunc(void );
#line 92
static void HplMsp430Usart0P__SIMO__selectModuleFunc(void );
# 89 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static volatile uint8_t HplMsp430Usart0P__IE1 __asm ("0x0000");
static volatile uint8_t HplMsp430Usart0P__ME1 __asm ("0x0004");
static volatile uint8_t HplMsp430Usart0P__IFG1 __asm ("0x0002");
static volatile uint8_t HplMsp430Usart0P__U0TCTL __asm ("0x0071");

static volatile uint8_t HplMsp430Usart0P__U0TXBUF __asm ("0x0077");

void sig_UART0RX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0012)))  ;




void sig_UART0TX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0010)))  ;
#line 132
static inline void HplMsp430Usart0P__Usart__setUbr(uint16_t control);










static inline void HplMsp430Usart0P__Usart__setUmctl(uint8_t control);







static inline void HplMsp430Usart0P__Usart__resetUsart(bool reset);
#line 207
static inline void HplMsp430Usart0P__Usart__disableUart(void );
#line 238
static inline void HplMsp430Usart0P__Usart__enableSpi(void );








static void HplMsp430Usart0P__Usart__disableSpi(void );








static inline void HplMsp430Usart0P__configSpi(msp430_spi_union_config_t *config);








static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 316
static inline bool HplMsp430Usart0P__Usart__isTxIntrPending(void );
#line 330
static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void );
#line 345
static inline void HplMsp430Usart0P__Usart__clrIntr(void );



static inline void HplMsp430Usart0P__Usart__disableRxIntr(void );







static inline void HplMsp430Usart0P__Usart__disableIntr(void );
#line 382
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data);



static inline uint8_t HplMsp430Usart0P__Usart__rx(void );
# 90 "/opt/tinyos-2.1.2/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void );
# 54 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x2b395b0a50c8, 
# 54 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x2b395b0a50c8);
# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(
# 40 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x2b395b0a4020);








static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(uint8_t id);
# 49 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1____nesc_unnamed4336 {
#line 49
  FcfsResourceQueueC__1__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[1U];
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );




static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );



static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
#line 82
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id);
# 53 "/opt/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(
# 55 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b395b0dc4b8);
# 61 "/opt/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(
# 55 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b395b0dc4b8);
# 65 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(
# 60 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b395b0da6e0);
# 59 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(
# 60 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b395b0da6e0);
# 79 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(resource_client_id_t id);
#line 53
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void );
#line 70
static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void );
# 73 "/opt/tinyos-2.1.2/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested(void );
#line 46
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void );
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(
# 54 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x2b395b0dd2f0);
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void );
# 75 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4337 {
#line 75
  ArbiterP__0__grantedTask = 9U
};
#line 75
typedef int /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_sillytask_grantedTask[/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask];
#line 67
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4338 {
#line 67
  ArbiterP__0__RES_CONTROLLED, ArbiterP__0__RES_GRANTING, ArbiterP__0__RES_IMM_GRANTING, ArbiterP__0__RES_BUSY
};
#line 68
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4339 {
#line 68
  ArbiterP__0__default_owner_id = 1U
};
#line 69
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4340 {
#line 69
  ArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;



static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(uint8_t id);
#line 93
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id);
#line 111
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id);
#line 133
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 153
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );
#line 166
static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );










static inline bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id);
#line 190
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
#line 202
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void );

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void );


static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void );


static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id);
# 97 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset);
# 49 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static volatile uint8_t HplMsp430I2C0P__U0CTL __asm ("0x0070");





static inline bool HplMsp430I2C0P__HplI2C__isI2C(void );



static inline void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
# 78 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420XC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput(void );
#line 73
static bool /*HplCC2420XC.CCAM*/Msp430GpioC__3__HplGeneralIO__get(void );
# 51 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420XC.CCAM*/Msp430GpioC__3__GeneralIO__get(void );
static inline void /*HplCC2420XC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void );
# 85 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420XC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput(void );
#line 48
static void /*HplCC2420XC.CSNM*/Msp430GpioC__4__HplGeneralIO__set(void );




static void /*HplCC2420XC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__set(void );
static inline void /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void );




static inline void /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void );
# 78 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420XC.FIFOM*/Msp430GpioC__5__HplGeneralIO__makeInput(void );
#line 73
static bool /*HplCC2420XC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get(void );
# 51 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420XC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void );
static inline void /*HplCC2420XC.FIFOM*/Msp430GpioC__5__GeneralIO__makeInput(void );
# 78 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__makeInput(void );
#line 73
static bool /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get(void );
# 51 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void );
static inline void /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__GeneralIO__makeInput(void );
# 85 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput(void );
#line 48
static void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set(void );




static void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void );
static inline void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void );




static inline void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void );
# 78 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420XC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput(void );
#line 73
static bool /*HplCC2420XC.SFDM*/Msp430GpioC__8__HplGeneralIO__get(void );
# 51 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420XC.SFDM*/Msp430GpioC__8__GeneralIO__get(void );
static inline void /*HplCC2420XC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void );
# 85 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420XC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput(void );
#line 48
static void /*HplCC2420XC.VRENM*/Msp430GpioC__9__HplGeneralIO__set(void );
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.VRENM*/Msp430GpioC__9__GeneralIO__set(void );





static inline void /*HplCC2420XC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void );
# 68 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430Capture__clearOverflow(void );
# 61 "/opt/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captured(uint16_t time);
# 55 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm);

static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__enableEvents(void );
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__disableEvents(void );
#line 44
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 99 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__GeneralIO__selectIOFunc(void );
#line 92
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__GeneralIO__selectModuleFunc(void );
# 49 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/GpioCaptureC.nc"
static error_t /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__enableCapture(uint8_t mode);
#line 61
static inline error_t /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captureRisingEdge(void );



static inline error_t /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captureFallingEdge(void );



static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__disable(void );






static inline void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 52 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__clear(void );
#line 47
static void /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__disable(void );
# 68 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__Interrupt__fired(void );
# 69 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__Interrupt__disable(void );







static inline void /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 72 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__fired(void );
#line 72
static void HplMsp430InterruptP__Port26__fired(void );
#line 72
static void HplMsp430InterruptP__Port17__fired(void );
#line 72
static void HplMsp430InterruptP__Port21__fired(void );
#line 72
static void HplMsp430InterruptP__Port12__fired(void );
#line 72
static void HplMsp430InterruptP__Port24__fired(void );
#line 72
static void HplMsp430InterruptP__Port15__fired(void );
#line 72
static void HplMsp430InterruptP__Port27__fired(void );
#line 72
static void HplMsp430InterruptP__Port10__fired(void );
#line 72
static void HplMsp430InterruptP__Port22__fired(void );
#line 72
static void HplMsp430InterruptP__Port13__fired(void );
#line 72
static void HplMsp430InterruptP__Port25__fired(void );
#line 72
static void HplMsp430InterruptP__Port16__fired(void );
#line 72
static void HplMsp430InterruptP__Port20__fired(void );
#line 72
static void HplMsp430InterruptP__Port11__fired(void );
#line 72
static void HplMsp430InterruptP__Port23__fired(void );
# 64 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
void sig_PORT1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0008)))  ;
#line 79
static inline void HplMsp430InterruptP__Port11__default__fired(void );
static inline void HplMsp430InterruptP__Port12__default__fired(void );
static inline void HplMsp430InterruptP__Port13__default__fired(void );
static inline void HplMsp430InterruptP__Port14__default__fired(void );
static inline void HplMsp430InterruptP__Port15__default__fired(void );
static inline void HplMsp430InterruptP__Port16__default__fired(void );
static inline void HplMsp430InterruptP__Port17__default__fired(void );








static inline void HplMsp430InterruptP__Port10__disable(void );







static inline void HplMsp430InterruptP__Port10__clear(void );
static inline void HplMsp430InterruptP__Port11__clear(void );
static inline void HplMsp430InterruptP__Port12__clear(void );
static inline void HplMsp430InterruptP__Port13__clear(void );
static inline void HplMsp430InterruptP__Port14__clear(void );
static inline void HplMsp430InterruptP__Port15__clear(void );
static inline void HplMsp430InterruptP__Port16__clear(void );
static inline void HplMsp430InterruptP__Port17__clear(void );
#line 169
void sig_PORT2_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0002)))  ;
#line 182
static inline void HplMsp430InterruptP__Port20__default__fired(void );
static inline void HplMsp430InterruptP__Port21__default__fired(void );
static inline void HplMsp430InterruptP__Port22__default__fired(void );
static inline void HplMsp430InterruptP__Port23__default__fired(void );
static inline void HplMsp430InterruptP__Port24__default__fired(void );
static inline void HplMsp430InterruptP__Port25__default__fired(void );
static inline void HplMsp430InterruptP__Port26__default__fired(void );
static inline void HplMsp430InterruptP__Port27__default__fired(void );
#line 206
static inline void HplMsp430InterruptP__Port20__clear(void );
static inline void HplMsp430InterruptP__Port21__clear(void );
static inline void HplMsp430InterruptP__Port22__clear(void );
static inline void HplMsp430InterruptP__Port23__clear(void );
static inline void HplMsp430InterruptP__Port24__clear(void );
static inline void HplMsp430InterruptP__Port25__clear(void );
static inline void HplMsp430InterruptP__Port26__clear(void );
static inline void HplMsp430InterruptP__Port27__clear(void );
# 41 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time);

static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void );
# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void );
# 57 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void );
#line 47
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare(void );










static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void );
#line 44
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void );
# 53 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Init__init(void );






static inline void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__start(uint16_t dt);




static inline void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void );




static inline void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );










static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 104
static inline uint16_t /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__getNow(void );









static inline void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static Msp430HybridAlarmCounterP__CounterMicro__size_type Msp430HybridAlarmCounterP__CounterMicro__get(void );
#line 82
static void Msp430HybridAlarmCounterP__Counter2ghz__overflow(void );
# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void Msp430HybridAlarmCounterP__Alarm2ghz__fired(void );
#line 103
static void Msp430HybridAlarmCounterP__AlarmMicro__startAt(Msp430HybridAlarmCounterP__AlarmMicro__size_type t0, Msp430HybridAlarmCounterP__AlarmMicro__size_type dt);
#line 88
static bool Msp430HybridAlarmCounterP__AlarmMicro__isRunning(void );
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static Msp430HybridAlarmCounterP__Counter32khz__size_type Msp430HybridAlarmCounterP__Counter32khz__get(void );






static bool Msp430HybridAlarmCounterP__Counter32khz__isOverflowPending(void );
# 50 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
uint32_t Msp430HybridAlarmCounterP__fireTime;


static __inline uint16_t Msp430HybridAlarmCounterP__now32khz(void );



static __inline uint16_t Msp430HybridAlarmCounterP__nowMicro(void );




static __inline void Msp430HybridAlarmCounterP__now(uint16_t *t32khz, uint16_t *tMicro, uint32_t *t2ghz);
#line 86
static __inline uint32_t Msp430HybridAlarmCounterP__now2ghz(void );
#line 101
static inline uint32_t Msp430HybridAlarmCounterP__Counter2ghz__get(void );








static inline bool Msp430HybridAlarmCounterP__Counter2ghz__isOverflowPending(void );
#line 124
static inline void Msp430HybridAlarmCounterP__Counter32khz__overflow(void );



static inline void Msp430HybridAlarmCounterP__CounterMicro__overflow(void );
#line 163
static inline void Msp430HybridAlarmCounterP__Alarm32khz__fired(void );
#line 176
static inline void Msp430HybridAlarmCounterP__AlarmMicro__fired(void );
#line 191
static inline void Msp430HybridAlarmCounterP__Alarm2ghz__default__fired(void );
#line 260
static inline mcu_power_t Msp430HybridAlarmCounterP__McuPowerOverride__lowestState(void );
# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__fired(void );
# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents(void );
# 70 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired(void );
#line 114
static inline void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow(void );
# 41 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEvent(uint16_t time);

static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__get(void );
# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__fired(void );
# 57 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__enableEvents(void );

static bool /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__areEventsEnabled(void );
#line 58
static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__disableEvents(void );
#line 44
static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__clearPendingInterrupt(void );
# 70 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__fired(void );





static inline bool /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__isRunning(void );




static inline void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 114
static inline void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__overflow(void );
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__size_type /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__get(void );






static bool /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__isOverflowPending(void );










static void /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__overflow(void );
# 67 "/opt/tinyos-2.1.2/tos/lib/timer/TransformCounterC.nc"
/*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__upper_count_type /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__m_upper;

enum /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1____nesc_unnamed4341 {

  TransformCounterC__1__LOW_SHIFT_RIGHT = 11, 
  TransformCounterC__1__HIGH_SHIFT_LEFT = 8 * sizeof(/*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__from_size_type ) - /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__LOW_SHIFT_RIGHT, 
  TransformCounterC__1__NUM_UPPER_BITS = 8 * sizeof(/*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_size_type ) - 8 * sizeof(/*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__from_size_type ) + 11, 



  TransformCounterC__1__OVERFLOW_MASK = /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__NUM_UPPER_BITS ? ((/*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__upper_count_type )2 << (/*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_size_type /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__get(void );
#line 133
static inline void /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__overflow(void );
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__get(void );
# 53 "/opt/tinyos-2.1.2/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get(void );




static inline void /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 110 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static void /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(
#line 103
message_t * msg, 






error_t error);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static error_t /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(
#line 67
message_t * msg, 







uint8_t len);
# 103 "/opt/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
static void /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(
#line 99
message_t * amsg, 



am_addr_t addr);
#line 162
static void /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(
#line 158
message_t * amsg, 



am_id_t t);
# 53 "/opt/tinyos-2.1.2/tos/system/AMQueueEntryP.nc"
static inline error_t /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err);
# 80 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(
# 48 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
am_id_t arg_0x2b395b468488, 
# 80 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 100 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(
# 46 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2b395b469318, 
# 96 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Packet.nc"
static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(
#line 74
message_t * msg);
#line 94
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(
#line 90
message_t * msg, 



uint8_t len);
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void );
# 78 "/opt/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(
#line 74
message_t * amsg);
#line 147
static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(
#line 143
message_t * amsg);
# 126 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4342 {
#line 126
  AMQueueImplP__0__CancelTask = 10U
};
#line 126
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_CancelTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask];
#line 169
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4343 {
#line 169
  AMQueueImplP__0__errorTask = 11U
};
#line 169
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_errorTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask];
#line 57
#line 55
typedef struct /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4344 {
  message_t * msg;
} /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t;

uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[1];
uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[1 / 8 + 1];

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void );
#line 90
static inline error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len);
#line 126
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
#line 163
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err);





static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );




static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );
#line 189
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err);
#line 215
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err);
# 397 "/opt/tinyos-2.1.2/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
  __eint();
}

# 137 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n)
{
}

# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x2b395a188458){
#line 39
  switch (arg_0x2b395a188458) {
#line 39
    case 0:
#line 39
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired();
#line 39
      break;
#line 39
    case 1:
#line 39
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired();
#line 39
      break;
#line 39
    case 2:
#line 39
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired();
#line 39
      break;
#line 39
    case 5:
#line 39
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired();
#line 39
      break;
#line 39
    default:
#line 39
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x2b395a188458);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 126 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(0);
}

# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA0__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired();
#line 39
}
#line 39
# 58 "/opt/tinyos-2.1.2/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 177 "/opt/tinyos-2.1.2/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void )
{
}

# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void ){
#line 82
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow();
#line 82
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 82
}
#line 82
# 133 "/opt/tinyos-2.1.2/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC__0__m_upper & /*CounterMilli32C.Transform*/TransformCounterC__0__OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow();
      }
  }
}

# 58 "/opt/tinyos-2.1.2/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void )
{
}

# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static void /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__overflow(void ){
#line 82
  /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow();
#line 82
}
#line 82
# 133 "/opt/tinyos-2.1.2/tos/lib/timer/TransformCounterC.nc"
static inline void /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__m_upper++;
    if ((/*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__m_upper & /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__OVERFLOW_MASK) == 0) {
      /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__overflow();
      }
  }
}

# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static void Msp430HybridAlarmCounterP__Counter2ghz__overflow(void ){
#line 82
  /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__overflow();
#line 82
}
#line 82
# 124 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
static inline void Msp430HybridAlarmCounterP__Counter32khz__overflow(void )
#line 124
{
  Msp430HybridAlarmCounterP__Counter2ghz__overflow();
}

# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 82
  Msp430HybridAlarmCounterP__Counter32khz__overflow();
#line 82
  /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow();
#line 82
}
#line 82
# 64 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow();
}

# 114 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void )
{
}

#line 114
static inline void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow(void )
{
}

# 196 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void )
{
}

#line 58
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4345 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(* (volatile uint16_t * )354U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n)
{
}

# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 81 "/opt/tinyos-2.1.2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void )
{
#line 82
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask();
}

# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 78
}
#line 78
# 162 "/opt/tinyos-2.1.2/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
      }
  }
}

# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired();
#line 78
}
#line 78
# 135 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__disableEvents(void )
{
  * (volatile uint16_t * )354U &= ~0x0010;
}

# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__disableEvents();
#line 58
}
#line 58
# 70 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired();
}

# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 45
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired();
#line 45
}
#line 45
# 97 "/opt/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP__isWaiting(uint8_t id)
{
  return SchedulerBasicP__m_next[id] != SchedulerBasicP__NO_TASK || SchedulerBasicP__m_tail == id;
}

static inline bool SchedulerBasicP__pushTask(uint8_t id)
{
  if (!SchedulerBasicP__isWaiting(id)) 
    {
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_head = id;
          SchedulerBasicP__m_tail = id;
        }
      else 
        {
          SchedulerBasicP__m_next[SchedulerBasicP__m_tail] = id;
          SchedulerBasicP__m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 49 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get();
}

# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 81 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__isOverflowPending(void )
{
  return * (volatile uint16_t * )352U & 1U;
}

# 46 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void ){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__isOverflowPending();
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 54 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending();
}

# 71 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 130 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__enableEvents(void )
{
  * (volatile uint16_t * )354U |= 0x0010;
}

# 57 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__enableEvents();
#line 57
}
#line 57
# 95 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )354U &= ~0x0001;
}

# 44 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 155 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )370U = x;
}

# 41 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(time);
#line 41
}
#line 41
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 165 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )370U = /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__get() + x;
}

# 43 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 81 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 87
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 94
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 97
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 99
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents();
  }
}

# 103 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4346 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(* (volatile uint16_t * )356U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n)
{
}

# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

#line 130
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void )
{
  * (volatile uint16_t * )390U |= 0x0010;
}

# 57 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents();
#line 57
}
#line 57
# 95 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )390U &= ~0x0001;
}

# 44 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 155 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )406U = x;
}

# 41 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(time);
#line 41
}
#line 41
# 62 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void )
{




  if (0) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 69
        {
          uint16_t t0;
          uint16_t t1 = * (volatile uint16_t * )400U;

#line 72
          do {
#line 72
              t0 = t1;
#line 72
              t1 = * (volatile uint16_t * )400U;
            }
          while (
#line 72
          t0 != t1);
          {
            unsigned int __nesc_temp = 
#line 73
            t1;

            {
#line 73
              __nesc_atomic_end(__nesc_atomic); 
#line 73
              return __nesc_temp;
            }
          }
        }
#line 76
        __nesc_atomic_end(__nesc_atomic); }
    }
  else 
#line 76
    {
      return * (volatile uint16_t * )400U;
    }
}

# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 165 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )406U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get() + x;
}

# 43 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 81 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 87
    if (elapsed >= dt) 
      {
        /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 94
        if (remaining <= 2) {
          /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 97
          /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 99
    /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__clearPendingInterrupt();
    /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__enableEvents();
  }
}

# 103 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void Msp430HybridAlarmCounterP__AlarmMicro__startAt(Msp430HybridAlarmCounterP__AlarmMicro__size_type t0, Msp430HybridAlarmCounterP__AlarmMicro__size_type dt){
#line 103
  /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 49 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get(void )
{
  return /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__get();
}

# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static Msp430HybridAlarmCounterP__CounterMicro__size_type Msp430HybridAlarmCounterP__CounterMicro__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 57 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
static __inline uint16_t Msp430HybridAlarmCounterP__nowMicro(void )
#line 57
{
  return Msp430HybridAlarmCounterP__CounterMicro__get();
}

# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static Msp430HybridAlarmCounterP__Counter32khz__size_type Msp430HybridAlarmCounterP__Counter32khz__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 53 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
static __inline uint16_t Msp430HybridAlarmCounterP__now32khz(void )
#line 53
{
  return Msp430HybridAlarmCounterP__Counter32khz__get();
}






static __inline void Msp430HybridAlarmCounterP__now(uint16_t *t32khz, uint16_t *tMicro, uint32_t *t2ghz)
#line 62
{
  uint16_t eMicro;

  /* atomic removed: atomic calls only */
#line 65
  {

    *t32khz = Msp430HybridAlarmCounterP__now32khz();


    *tMicro = Msp430HybridAlarmCounterP__nowMicro();


    while (*t32khz == Msp430HybridAlarmCounterP__now32khz()) ;
  }


  eMicro = Msp430HybridAlarmCounterP__nowMicro() - *tMicro;


  *t2ghz = (uint32_t )*t32khz << 16;


  *t2ghz -= (uint32_t )eMicro << 11;
}

#line 163
static inline void Msp430HybridAlarmCounterP__Alarm32khz__fired(void )
#line 163
{
  uint16_t tMicro;
#line 164
  uint16_t t32khz;
  uint32_t t2ghz;
#line 165
  uint32_t dt;


  Msp430HybridAlarmCounterP__now(&t32khz, &tMicro, &t2ghz);


  dt = Msp430HybridAlarmCounterP__fireTime - t2ghz;

  Msp430HybridAlarmCounterP__AlarmMicro__startAt(tMicro, dt >> 11);
}

# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__fired(void ){
#line 78
  Msp430HybridAlarmCounterP__Alarm32khz__fired();
#line 78
}
#line 78
# 135 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__disableEvents(void )
{
  * (volatile uint16_t * )356U &= ~0x0010;
}

# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__disableEvents();
#line 58
}
#line 58
# 70 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired(void )
{
  /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents();
  /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__fired();
}

# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 45
  /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired();
#line 45
}
#line 45
# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4347 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(* (volatile uint16_t * )358U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n)
{
}

# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void )
{
}

# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired();
#line 45
}
#line 45
# 131 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 134
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(n >> 1);
}

# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA1__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired();
#line 39
}
#line 39
# 126 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(0);
}

# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB0__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired();
#line 39
}
#line 39
# 196 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void )
{
}

# 83 "/opt/tinyos-2.1.2/tos/lib/timer/BusyWaitCounterC.nc"
static inline void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow(void )
{
}

# 128 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
static inline void Msp430HybridAlarmCounterP__CounterMicro__overflow(void )
#line 128
{
}

# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__overflow(void ){
#line 82
  Msp430HybridAlarmCounterP__CounterMicro__overflow();
#line 82
  /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow();
#line 82
}
#line 82
# 64 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void )
{
  /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__overflow();
}

# 114 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__overflow(void )
{
}

#line 114
static inline void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void )
{
}

# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 48
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow();
#line 48
  /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__overflow();
#line 48
  /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow();
#line 48
}
#line 48
# 137 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow();
}

# 59 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
inline static void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Tasklet__schedule(void ){
#line 59
  TaskletC__Tasklet__schedule();
#line 59
}
#line 59
# 65 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
static inline void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__state == /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__STATE_WAIT) {
      /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__state = /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__STATE_FIRED;
      }
  }
  /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Tasklet__schedule();
}

# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void ){
#line 78
  /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__fired();
#line 78
}
#line 78
# 135 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents();
#line 58
}
#line 58
# 70 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void )
{
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired();
}

# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 45
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired();
#line 45
}
#line 45
# 150 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n)
{
}

# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4348 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(* (volatile uint16_t * )386U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired();
    }
}

# 59 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
inline static void CC2420XDriverLayerP__Tasklet__schedule(void ){
#line 59
  TaskletC__Tasklet__schedule();
#line 59
}
#line 59
# 61 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline error_t /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captureRisingEdge(void )
#line 61
{
  return /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__enableCapture(MSP430TIMER_CM_RISING);
}

# 53 "/opt/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420XDriverLayerP__SfdCapture__captureRisingEdge(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captureRisingEdge();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 148 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static inline void CC2420XDriverLayerP__RadioAlarm__fired(void )
{
  if (CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_PD_2_IDLE) {
      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_IDLE;
      if (CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_STANDBY) {
        CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_SIGNAL_DONE;
        }
    }
  else {
#line 155
    if (CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_IDLE_2_RX_ON) {
        CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_RX_ON;
        CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_SIGNAL_DONE;

        CC2420XDriverLayerP__SfdCapture__captureRisingEdge();
      }
    else {
      for (; 0; ) ;
      }
    }
  CC2420XDriverLayerP__Tasklet__schedule();
}

# 56 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
inline static void /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioSend__sendDone(error_t error){
#line 56
  /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioSend__sendDone(error);
#line 56
}
#line 56
# 155 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/RandomCollisionLayerP.nc"
static inline void /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubSend__sendDone(error_t error)
{
  for (; 0; ) ;

  /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__state = /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__STATE_READY;
  /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioSend__sendDone(error);
}

# 56 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
inline static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioSend__sendDone(error_t error){
#line 56
  /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubSend__sendDone(error);
#line 56
}
#line 56
# 129 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline void CC2420XRadioP__SoftwareAckConfig__reportChannelError(void )
{
}

# 86 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
inline static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__reportChannelError(void ){
#line 86
  CC2420XRadioP__SoftwareAckConfig__reportChannelError();
#line 86
}
#line 86
# 124 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckLayerC.nc"
static inline void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioAlarm__fired(void )
{
  for (; 0; ) ;

  /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__reportChannelError();

  /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__state = /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__STATE_READY;
  /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioSend__sendDone(SUCCESS);
}

# 239 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline uint16_t CC2420XRadioP__RandomCollisionConfig__getCongestionBackoff(message_t *msg)
{
  return (uint16_t )(2240 * 1);
}

# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/RandomCollisionConfig.nc"
inline static uint16_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__Config__getCongestionBackoff(message_t *msg){
#line 46
  unsigned int __nesc_result;
#line 46

#line 46
  __nesc_result = CC2420XRadioP__RandomCollisionConfig__getCongestionBackoff(msg);
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 72 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline uint8_t CC2420XRadioP__CC2420XDriverConfig__metadataLength(message_t *msg)
{
  return 0;
}

# 41 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverConfig.nc"
inline static uint8_t CC2420XDriverLayerP__Config__metadataLength(message_t *msg){
#line 41
  unsigned char __nesc_result;
#line 41

#line 41
  __nesc_result = CC2420XRadioP__CC2420XDriverConfig__metadataLength(msg);
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 1207 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static inline uint8_t CC2420XDriverLayerP__RadioPacket__metadataLength(message_t *msg)
{
  return CC2420XDriverLayerP__Config__metadataLength(msg) + sizeof(cc2420x_metadata_t );
}

# 65 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__SubPacket__metadataLength(message_t *msg){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = CC2420XDriverLayerP__RadioPacket__metadataLength(msg);
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 110 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MetadataFlagsLayerC.nc"
static inline uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__metadataLength(message_t *msg)
{
  return /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__SubPacket__metadataLength(msg) + sizeof(flags_metadata_t );
}

#line 54
static inline flags_metadata_t */*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__getMeta(message_t *msg)
{
  return (void *)msg + sizeof(message_t ) - /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__metadataLength(msg);
}

#line 73
static inline void /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__PacketFlag__clear(uint8_t bit, message_t *msg)
{
  for (; 0; ) ;

  /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__getMeta(msg)->flags &= ~(1 << bit);
}

# 55 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
inline static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__AckReceivedFlag__clear(message_t *msg){
#line 55
  /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__PacketFlag__clear(0U, msg);
#line 55
}
#line 55
# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
inline static error_t /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubSend__send(message_t *msg){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = CC2420XDriverLayerP__RadioSend__send(msg);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 78 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckLayerC.nc"
static inline error_t /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioSend__send(message_t *msg)
{
  error_t error;

  if (/*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__state == /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__STATE_READY) 
    {
      if ((error = /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubSend__send(msg)) == SUCCESS) 
        {
          /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__AckReceivedFlag__clear(msg);
          /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__state = /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__STATE_DATA_SEND;
          /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__txMsg = msg;
        }
    }
  else {
    error = EBUSY;
    }
  return error;
}

# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
inline static error_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubSend__send(message_t *msg){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioSend__send(msg);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
#line 63
inline static void /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioSend__ready(void ){
#line 63
  /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioSend__ready();
#line 63
}
#line 63
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 104 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline uint16_t /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__getNow(void )
{
  return /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get();
}

# 109 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__size_type /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__getNow(void ){
#line 109
  unsigned int __nesc_result;
#line 109

#line 109
  __nesc_result = /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__getNow();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 76 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
static __inline tradio_size /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__getNow(uint8_t id)
{
  return /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__getNow();
}

# 65 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
inline static tradio_size /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__getNow(void ){
#line 65
  unsigned int __nesc_result;
#line 65

#line 65
  __nesc_result = /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__getNow(1U);
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 110 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/RandomCollisionLayerP.nc"
static inline void /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__fired(void )
{
  error_t error;
  int16_t delay;

  for (; 0; ) ;

  delay = (int16_t )/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__txBarrier - /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__getNow();

  if (/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__state == /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__STATE_BARRIER) 
    {
      /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__state = /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__STATE_READY;

      /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioSend__ready();
      return;
    }
  else {
#line 126
    if (/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__state & /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__STATE_BARRIER && delay > 0) {
      error = EBUSY;
      }
    else {
#line 129
      error = /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubSend__send(/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__txMsg);
      }
    }
#line 131
  if (error != SUCCESS) 
    {
      if ((/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__state & ~/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__STATE_BARRIER) == /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__STATE_TX_PENDING_FIRST) 
        {
          /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__state = (/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__state & /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__STATE_BARRIER) | /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__STATE_TX_PENDING_SECOND;
          /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__wait(/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__getBackoff(/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__Config__getCongestionBackoff(/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__txMsg)));
        }
      else 
        {
          if (/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__state & /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__STATE_BARRIER && delay > 0) 
            {
              /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__state = /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__STATE_BARRIER;
              /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__wait(delay);
            }
          else {
            /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__state = /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__STATE_READY;
            }
          /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioSend__sendDone(error);
        }
    }
  else {
    /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__state = /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__STATE_TX_SENDING;
    }
}

# 262 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline void CC2420XRadioP__RadioAlarm__fired(void )
{
}

# 90 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
static inline void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__default__fired(uint8_t id)
{
}

# 60 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
inline static void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__fired(uint8_t arg_0x2b395a8ab220){
#line 60
  switch (arg_0x2b395a8ab220) {
#line 60
    case 0U:
#line 60
      CC2420XRadioP__RadioAlarm__fired();
#line 60
      break;
#line 60
    case 1U:
#line 60
      /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__fired();
#line 60
      break;
#line 60
    case 2U:
#line 60
      /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioAlarm__fired();
#line 60
      break;
#line 60
    case 3U:
#line 60
      CC2420XDriverLayerP__RadioAlarm__fired();
#line 60
      break;
#line 60
    default:
#line 60
      /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__default__fired(arg_0x2b395a8ab220);
#line 60
      break;
#line 60
    }
#line 60
}
#line 60
# 81 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
static inline void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Tasklet__run(void )
{
  if (/*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__state == /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__STATE_FIRED) 
    {
      /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__state = /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__STATE_READY;
      /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__fired(/*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__alarm);
    }
}

# 261 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MessageBufferLayerP.nc"
static inline void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Tasklet__run(void )
{
}

# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420XDriverLayerP__releaseSpi__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420XDriverLayerP__releaseSpi);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 94 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
static __inline bool /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__isFree(uint8_t id)
{
  return /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__state == /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__STATE_READY;
}

# 45 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
inline static bool /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__isFree(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__isFree(1U);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 71 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/RandomCollisionLayerP.nc"
static inline void /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubSend__ready(void )
{
  if (/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__state == /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__STATE_READY && /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__isFree()) {
    /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioSend__ready();
    }
}

# 63 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
inline static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioSend__ready(void ){
#line 63
  /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubSend__ready();
#line 63
}
#line 63
# 72 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckLayerC.nc"
static inline void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubSend__ready(void )
{
  if (/*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__state == /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__STATE_READY) {
    /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioSend__ready();
    }
}

# 63 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
inline static void CC2420XDriverLayerP__RadioSend__ready(void ){
#line 63
  /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubSend__ready();
#line 63
}
#line 63
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__stateDoneTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__stateDoneTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 163 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MessageBufferLayerP.nc"
static inline void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioState__done(void )
{
  /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__stateDoneTask__postTask();
}

# 69 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioState.nc"
inline static void CC2420XDriverLayerP__RadioState__done(void ){
#line 69
  /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioState__done();
#line 69
}
#line 69
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.CSNM*/Msp430GpioC__4__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set();
#line 48
}
#line 48
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__set(void )
#line 48
{
#line 48
  /*HplCC2420XC.CSNM*/Msp430GpioC__4__HplGeneralIO__set();
}

# 40 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__CSN__set(void ){
#line 40
  /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__set();
#line 40
}
#line 40
# 386 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline uint8_t HplMsp430Usart0P__Usart__rx(void )
#line 386
{
  return U0RXBUF;
}

# 231 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx(void ){
#line 231
  unsigned char __nesc_result;
#line 231

#line 231
  __nesc_result = HplMsp430Usart0P__Usart__rx();
#line 231

#line 231
  return __nesc_result;
#line 231
}
#line 231
# 330 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void )
#line 330
{
  if (HplMsp430Usart0P__IFG1 & 0x40) {
      return TRUE;
    }
  return FALSE;
}

# 192 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending(void ){
#line 192
  unsigned char __nesc_result;
#line 192

#line 192
  __nesc_result = HplMsp430Usart0P__Usart__isRxIntrPending();
#line 192

#line 192
  return __nesc_result;
#line 192
}
#line 192
# 150 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static __inline uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitRead(void )
#line 150
{
  while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending()) ;
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx();
}

# 68 "/opt/tinyos-2.1.2/tos/interfaces/FastSpiByte.nc"
inline static uint8_t CC2420XDriverLayerP__FastSpiByte__splitRead(void ){
#line 68
  unsigned char __nesc_result;
#line 68

#line 68
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitRead();
#line 68

#line 68
  return __nesc_result;
#line 68
}
#line 68
# 382 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data)
#line 382
{
  HplMsp430Usart0P__U0TXBUF = data;
}

# 224 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(uint8_t data){
#line 224
  HplMsp430Usart0P__Usart__tx(data);
#line 224
}
#line 224
# 316 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline bool HplMsp430Usart0P__Usart__isTxIntrPending(void )
#line 316
{
  if (HplMsp430Usart0P__IFG1 & 0x80) {
      return TRUE;
    }
  return FALSE;
}

# 187 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isTxIntrPending(void ){
#line 187
  unsigned char __nesc_result;
#line 187

#line 187
  __nesc_result = HplMsp430Usart0P__Usart__isTxIntrPending();
#line 187

#line 187
  return __nesc_result;
#line 187
}
#line 187
# 155 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static __inline uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitReadWrite(uint8_t data)
#line 155
{
  uint8_t b;

  while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending()) ;
  b = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx();

  while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isTxIntrPending()) ;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(data);

  return b;
}

# 74 "/opt/tinyos-2.1.2/tos/interfaces/FastSpiByte.nc"
inline static uint8_t CC2420XDriverLayerP__FastSpiByte__splitReadWrite(uint8_t data){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitReadWrite(data);
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 146 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static __inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitWrite(uint8_t data)
#line 146
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(data);
}

# 62 "/opt/tinyos-2.1.2/tos/interfaces/FastSpiByte.nc"
inline static void CC2420XDriverLayerP__FastSpiByte__splitWrite(uint8_t data){
#line 62
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitWrite(data);
#line 62
}
#line 62
# 53 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr();
#line 53
}
#line 53
# 49 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void )
#line 49
{
#line 49
  /*HplCC2420XC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr();
}

# 41 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__CSN__clr(void ){
#line 41
  /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__clr();
#line 41
}
#line 41
# 211 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static __inline cc2420X_status_t CC2420XDriverLayerP__writeRegister(uint8_t reg, uint16_t value)
{
  cc2420X_status_t status;

  for (; 0; ) ;
  for (; 0; ) ;

  CC2420XDriverLayerP__CSN__set();
  CC2420XDriverLayerP__CSN__clr();

  CC2420XDriverLayerP__FastSpiByte__splitWrite(CC2420X_CMD_REGISTER_WRITE | reg);
  CC2420XDriverLayerP__FastSpiByte__splitReadWrite(value >> 8);
  CC2420XDriverLayerP__FastSpiByte__splitReadWrite(value & 0xff);
  status.value = CC2420XDriverLayerP__FastSpiByte__splitRead();

  CC2420XDriverLayerP__CSN__set();
  return status;
}

#line 463
static __inline void CC2420XDriverLayerP__setChannel(void )
{
  cc2420X_fsctrl_t fsctrl;

  fsctrl = cc2420X_fsctrl_default;
  fsctrl.f.freq = 357 + 5 * (CC2420XDriverLayerP__channel - 11);

  CC2420XDriverLayerP__writeRegister(CC2420X_FSCTRL, fsctrl.value);
}

static __inline void CC2420XDriverLayerP__changeChannel(void )
{
  for (; 0; ) ;
  for (; 0; ) ;

  if (CC2420XDriverLayerP__isSpiAcquired()) 
    {
      CC2420XDriverLayerP__setChannel();

      if (CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_RX_ON) {
          CC2420XDriverLayerP__RadioAlarm__wait(IDLE_2_RX_ON_TIME);
          CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_IDLE_2_RX_ON;
        }
      else {
        CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_SIGNAL_DONE;
        }
    }
}

# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set();
#line 48
}
#line 48
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void )
#line 48
{
#line 48
  /*HplCC2420XC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set();
}

# 40 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__RSTN__set(void ){
#line 40
  /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__set();
#line 40
}
#line 40
# 53 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr();
#line 53
}
#line 53
# 49 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void )
#line 49
{
#line 49
  /*HplCC2420XC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr();
}

# 41 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__RSTN__clr(void ){
#line 41
  /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__clr();
#line 41
}
#line 41
# 373 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static __inline void CC2420XDriverLayerP__resetRadio(void )
#line 373
{

  cc2420X_iocfg0_t iocfg0;
  cc2420X_mdmctrl0_t mdmctrl0;


  CC2420XDriverLayerP__RSTN__clr();
  CC2420XDriverLayerP__RSTN__set();


  iocfg0 = cc2420X_iocfg0_default;
  iocfg0.f.fifop_thr = 127;
  CC2420XDriverLayerP__writeRegister(CC2420X_IOCFG0, iocfg0.value);


  mdmctrl0 = cc2420X_mdmctrl0_default;
  mdmctrl0.f.reserved_frame_mode = 1;
  mdmctrl0.f.adr_decode = 0;
  CC2420XDriverLayerP__writeRegister(CC2420X_MDMCTRL0, mdmctrl0.value);

  CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_PD;
}


static inline void CC2420XDriverLayerP__initRadio(void )
{
  CC2420XDriverLayerP__resetRadio();

  CC2420XDriverLayerP__txPower = 31 & CC2420X_TX_PWR_MASK;
  CC2420XDriverLayerP__channel = 11 & CC2420X_CHANNEL_MASK;
}

#line 189
static __inline cc2420X_status_t CC2420XDriverLayerP__strobe(uint8_t reg)
{
  cc2420X_status_t status;

  for (; 0; ) ;
  for (; 0; ) ;

  CC2420XDriverLayerP__CSN__set();
  CC2420XDriverLayerP__CSN__clr();

  CC2420XDriverLayerP__FastSpiByte__splitWrite(CC2420X_CMD_REGISTER_WRITE | reg);
  status.value = CC2420XDriverLayerP__FastSpiByte__splitRead();

  CC2420XDriverLayerP__CSN__set();
  return status;
}

# 66 "/opt/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
inline static void CC2420XDriverLayerP__SfdCapture__disable(void ){
#line 66
  /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__disable();
#line 66
}
#line 66
# 45 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
inline static bool CC2420XDriverLayerP__RadioAlarm__isFree(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__isFree(3U);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 493 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static __inline void CC2420XDriverLayerP__changeState(void )
{


  if ((
#line 496
  CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_STANDBY || CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_TURNON)
   && CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_PD && CC2420XDriverLayerP__isSpiAcquired() && CC2420XDriverLayerP__RadioAlarm__isFree()) 
    {

      CC2420XDriverLayerP__strobe(CC2420X_SXOSCON);

      CC2420XDriverLayerP__RadioAlarm__wait(PD_2_IDLE_TIME);
      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_PD_2_IDLE;
    }
  else {
#line 505
    if (CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_TURNON && CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_IDLE && CC2420XDriverLayerP__isSpiAcquired() && CC2420XDriverLayerP__RadioAlarm__isFree()) 
      {

        CC2420XDriverLayerP__setChannel();


        CC2420XDriverLayerP__strobe(CC2420X_SRXON);
        CC2420XDriverLayerP__RadioAlarm__wait(IDLE_2_RX_ON_TIME);
        CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_IDLE_2_RX_ON;
      }
    else {
      if ((
#line 515
      CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_TURNOFF || CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_STANDBY)
       && CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_RX_ON && CC2420XDriverLayerP__isSpiAcquired()) 
        {

          CC2420XDriverLayerP__SfdCapture__disable();


          CC2420XDriverLayerP__strobe(CC2420X_SRFOFF);
          CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_IDLE;
        }
      }
    }
#line 526
  if (CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_TURNOFF && CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_IDLE && CC2420XDriverLayerP__isSpiAcquired()) 
    {

      CC2420XDriverLayerP__strobe(CC2420X_SXOSCOFF);


      CC2420XDriverLayerP__initRadio();
      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_PD;
      CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_SIGNAL_DONE;
    }
  else {
#line 536
    if (CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_STANDBY && CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_IDLE) {
      CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_SIGNAL_DONE;
      }
    }
}

# 297 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_leuint8(void * target, uint8_t value)
#line 297
{
  uint8_t *base = target;

#line 299
  base[0] = value;
  return value;
}

# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__deliverTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__deliverTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 351 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MessageBufferLayerP.nc"
static inline message_t */*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioReceive__receive(message_t *msg)
{
  message_t *m;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if (/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueueSize >= /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RECEIVE_QUEUE_SIZE) {
        m = msg;
        }
      else {
          uint8_t idx = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueueHead + /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueueSize;

#line 362
          if (idx >= /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RECEIVE_QUEUE_SIZE) {
            idx -= /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RECEIVE_QUEUE_SIZE;
            }
          m = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueue[idx];
          /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueue[idx] = msg;

          ++/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueueSize;
          /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__deliverTask__postTask();
        }
    }
#line 371
    __nesc_atomic_end(__nesc_atomic); }

  return m;
}

# 53 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
inline static message_t */*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__RadioReceive__receive(message_t *msg){
#line 53
  nx_struct message_t *__nesc_result;
#line 53

#line 53
  __nesc_result = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioReceive__receive(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 163 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/NeighborhoodP.nc"
static __inline void NeighborhoodP__NeighborhoodFlag__set(uint8_t bit, uint8_t idx)
{
  NeighborhoodP__flags[idx] |= 1 << bit;
}

# 51 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/NeighborhoodFlag.nc"
inline static void /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__NeighborhoodFlag__set(uint8_t idx){
#line 51
  NeighborhoodP__NeighborhoodFlag__set(0U, idx);
#line 51
}
#line 51
# 153 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline void CC2420XRadioP__UniqueConfig__reportChannelError(void )
{
}

# 58 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/UniqueConfig.nc"
inline static void /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__UniqueConfig__reportChannelError(void ){
#line 58
  CC2420XRadioP__UniqueConfig__reportChannelError();
#line 58
}
#line 58
# 158 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/NeighborhoodP.nc"
static __inline bool NeighborhoodP__NeighborhoodFlag__get(uint8_t bit, uint8_t idx)
{
  return NeighborhoodP__flags[idx] & (1 << bit);
}

# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/NeighborhoodFlag.nc"
inline static bool /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__NeighborhoodFlag__get(uint8_t idx){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = NeighborhoodP__NeighborhoodFlag__get(0U, idx);
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 292 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_leuint8(const void * source)
#line 292
{
  const uint8_t *base = source;

#line 294
  return base[0];
}

# 62 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline uint8_t CC2420XRadioP__CC2420XDriverConfig__headerLength(message_t *msg)
{
  return (unsigned short )& ((message_t *)0)->data - sizeof(cc2420xpacket_header_t );
}

# 29 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverConfig.nc"
inline static uint8_t CC2420XDriverLayerP__Config__headerLength(message_t *msg){
#line 29
  unsigned char __nesc_result;
#line 29

#line 29
  __nesc_result = CC2420XRadioP__CC2420XDriverConfig__headerLength(msg);
#line 29

#line 29
  return __nesc_result;
#line 29
}
#line 29
# 1181 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static inline uint8_t CC2420XDriverLayerP__RadioPacket__headerLength(message_t *msg)
{
  return CC2420XDriverLayerP__Config__headerLength(msg) + sizeof(cc2420x_header_t );
}

# 43 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__SubPacket__headerLength(message_t *msg){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = CC2420XDriverLayerP__RadioPacket__headerLength(msg);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 90 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MetadataFlagsLayerC.nc"
static inline uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__headerLength(message_t *msg)
{
  return /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__SubPacket__headerLength(msg);
}

# 43 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__SubPacket__headerLength(message_t *msg){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__headerLength(msg);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 116 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TimeStampingLayerP.nc"
static inline uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__headerLength(message_t *msg)
{
  return /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__SubPacket__headerLength(msg);
}

# 43 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__headerLength(message_t *msg){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__headerLength(msg);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 77 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline ieee154_simple_header_t */*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(message_t *msg)
{
  return (void *)msg + /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__headerLength(msg);
}

#line 158
static inline uint8_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDSN(message_t *msg)
{
  return __nesc_ntoh_leuint8(/*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->dsn.nxdata);
}

# 120 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static uint8_t CC2420XRadioP__Ieee154PacketLayer__getDSN(message_t *msg){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDSN(msg);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 138 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline uint8_t CC2420XRadioP__UniqueConfig__getSequenceNumber(message_t *msg)
{
  return CC2420XRadioP__Ieee154PacketLayer__getDSN(msg);
}

# 42 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/UniqueConfig.nc"
inline static uint8_t /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__UniqueConfig__getSequenceNumber(message_t *msg){
#line 42
  unsigned char __nesc_result;
#line 42

#line 42
  __nesc_result = CC2420XRadioP__UniqueConfig__getSequenceNumber(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 322 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_leuint16(const void * source)
#line 322
{
  const uint8_t *base = source;

#line 324
  return ((uint16_t )base[1] << 8) | base[0];
}

# 188 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline uint16_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getSrcAddr(message_t *msg)
{
  return __nesc_ntoh_leuint16(/*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->src.nxdata);
}

# 151 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static uint16_t CC2420XRadioP__Ieee154PacketLayer__getSrcAddr(message_t *msg){
#line 151
  unsigned int __nesc_result;
#line 151

#line 151
  __nesc_result = /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getSrcAddr(msg);
#line 151

#line 151
  return __nesc_result;
#line 151
}
#line 151
# 148 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline am_addr_t CC2420XRadioP__UniqueConfig__getSender(message_t *msg)
{
  return CC2420XRadioP__Ieee154PacketLayer__getSrcAddr(msg);
}

# 47 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/UniqueConfig.nc"
inline static am_addr_t /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__UniqueConfig__getSender(message_t *msg){
#line 47
  unsigned int __nesc_result;
#line 47

#line 47
  __nesc_result = CC2420XRadioP__UniqueConfig__getSender(msg);
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 116 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/UniqueLayerP.nc"
static inline void /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__Neighborhood__evicted(uint8_t idx)
#line 116
{
}

# 80 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/Neighborhood.nc"
inline static void NeighborhoodP__Neighborhood__evicted(uint8_t idx){
#line 80
  /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__Neighborhood__evicted(idx);
#line 80
}
#line 80
# 94 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/NeighborhoodP.nc"
static inline uint8_t NeighborhoodP__Neighborhood__insertNode(am_addr_t node)
{
  uint8_t i;
  uint8_t maxAge;

  if (NeighborhoodP__nodes[NeighborhoodP__last] == node) 
    {
      if (NeighborhoodP__ages[NeighborhoodP__last] == NeighborhoodP__time) {
        return NeighborhoodP__last;
        }
      NeighborhoodP__ages[NeighborhoodP__last] = ++NeighborhoodP__time;
      maxAge = 0x80;
    }
  else 
    {
      uint8_t oldest = 0;

#line 110
      maxAge = 0;

      for (i = 0; i < 5; ++i) 
        {
          uint8_t age;

          if (NeighborhoodP__nodes[i] == node) 
            {
              NeighborhoodP__last = i;
              if (NeighborhoodP__ages[i] == NeighborhoodP__time) {
                return i;
                }
              NeighborhoodP__ages[i] = ++NeighborhoodP__time;
              maxAge = 0x80;
              break;
            }

          age = NeighborhoodP__time - NeighborhoodP__ages[i];
          if (age > maxAge) 
            {
              maxAge = age;
              oldest = i;
            }
        }

      if (i == 5) 
        {
          NeighborhoodP__Neighborhood__evicted(oldest);

          NeighborhoodP__last = oldest;
          NeighborhoodP__nodes[oldest] = node;
          NeighborhoodP__ages[oldest] = ++NeighborhoodP__time;
          NeighborhoodP__flags[oldest] = 0;
        }
    }

  if ((NeighborhoodP__time & 0x7F) == 0x7F && maxAge >= 0x7F) 
    {
      for (i = 0; i < 5; ++i) 
        {
          if ((NeighborhoodP__ages[i] | 0x7F) != NeighborhoodP__time) {
            NeighborhoodP__ages[i] = NeighborhoodP__time & 0x80;
            }
        }
    }
  return NeighborhoodP__last;
}

# 71 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/Neighborhood.nc"
inline static uint8_t /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__Neighborhood__insertNode(am_addr_t id){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = NeighborhoodP__Neighborhood__insertNode(id);
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 93 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/UniqueLayerP.nc"
static inline message_t */*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__SubReceive__receive(message_t *msg)
{
  uint8_t idx = /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__Neighborhood__insertNode(/*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__UniqueConfig__getSender(msg));
  uint8_t dsn = /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__UniqueConfig__getSequenceNumber(msg);

  if (/*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__NeighborhoodFlag__get(idx)) 
    {
      uint8_t diff = dsn - /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__receivedNumbers[idx];

      if (diff == 0) 
        {
          /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__UniqueConfig__reportChannelError();
          return msg;
        }
    }
  else {
    /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__NeighborhoodFlag__set(idx);
    }
  /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__receivedNumbers[idx] = dsn;

  return /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__RadioReceive__receive(msg);
}

# 53 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
inline static message_t */*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioReceive__receive(message_t *msg){
#line 53
  nx_struct message_t *__nesc_result;
#line 53

#line 53
  __nesc_result = /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__SubReceive__receive(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 168 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static bool CC2420XRadioP__Ieee154PacketLayer__requiresAckReply(message_t *msg){
#line 168
  unsigned char __nesc_result;
#line 168

#line 168
  __nesc_result = /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__requiresAckReply(msg);
#line 168

#line 168
  return __nesc_result;
#line 168
}
#line 168
# 65 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
inline static tradio_size CC2420XRadioP__RadioAlarm__getNow(void ){
#line 65
  unsigned int __nesc_result;
#line 65

#line 65
  __nesc_result = /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__getNow(0U);
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 246 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline uint16_t CC2420XRadioP__RandomCollisionConfig__getTransmitBarrier(message_t *msg)
{
  uint16_t time;


  time = CC2420XRadioP__RadioAlarm__getNow();


  if (CC2420XRadioP__Ieee154PacketLayer__requiresAckReply(msg)) {
    time += (uint16_t )(32 * (-5 + 16 + 11 + 5) * 1);
    }
  else {
#line 257
    time += (uint16_t )(32 * (-5 + 5) * 1);
    }
  return time;
}

# 57 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/RandomCollisionConfig.nc"
inline static uint16_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__Config__getTransmitBarrier(message_t *msg){
#line 57
  unsigned int __nesc_result;
#line 57

#line 57
  __nesc_result = CC2420XRadioP__RandomCollisionConfig__getTransmitBarrier(msg);
#line 57

#line 57
  return __nesc_result;
#line 57
}
#line 57
# 168 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/RandomCollisionLayerP.nc"
static inline message_t */*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubReceive__receive(message_t *msg)
{
  int16_t delay;

  /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__txBarrier = /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__Config__getTransmitBarrier(msg);
  delay = /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__txBarrier - /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__getNow();

  if (delay > 0) 
    {
      if (/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__state == /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__STATE_READY) 
        {

          if (/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__isFree()) 
            {
              /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__wait(delay);
              /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__state = /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__STATE_BARRIER;
            }
        }
      else {
        /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__state |= /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__STATE_BARRIER;
        }
    }
  return /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioReceive__receive(msg);
}

# 53 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
inline static message_t */*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioReceive__receive(message_t *msg){
#line 53
  nx_struct message_t *__nesc_result;
#line 53

#line 53
  __nesc_result = /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubReceive__receive(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 327 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_hton_leuint16(void * target, uint16_t value)
#line 327
{
  uint8_t *base = target;

#line 329
  base[0] = value;
  base[1] = value >> 8;
  return value;
}

# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static void /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__SubPacket__setPayloadLength(message_t *msg, uint8_t length){
#line 54
  CC2420XDriverLayerP__RadioPacket__setPayloadLength(msg, length);
#line 54
}
#line 54
# 100 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MetadataFlagsLayerC.nc"
static inline void /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__setPayloadLength(message_t *msg, uint8_t length)
{
  /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__SubPacket__setPayloadLength(msg, length);
}

# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static void /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__SubPacket__setPayloadLength(message_t *msg, uint8_t length){
#line 54
  /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__setPayloadLength(msg, length);
#line 54
}
#line 54
# 126 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TimeStampingLayerP.nc"
static inline void /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__setPayloadLength(message_t *msg, uint8_t length)
{
  /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__SubPacket__setPayloadLength(msg, length);
}

# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__setPayloadLength(message_t *msg, uint8_t length){
#line 54
  /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__setPayloadLength(msg, length);
#line 54
}
#line 54
# 115 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__createAckReply(message_t *data, message_t *ack)
{
  ieee154_simple_header_t *header = /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(ack);

#line 118
  /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__setPayloadLength(ack, /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_ACK_FRAME_LENGTH);

  __nesc_hton_leuint16(header->fcf.nxdata, /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_ACK_FRAME_VALUE);
  __nesc_hton_leuint8(header->dsn.nxdata, __nesc_ntoh_leuint8(/*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(data)->dsn.nxdata));
}

# 88 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static void CC2420XRadioP__Ieee154PacketLayer__createAckReply(message_t *data, message_t *ack){
#line 88
  /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__createAckReply(data, ack);
#line 88
}
#line 88
# 115 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline void CC2420XRadioP__SoftwareAckConfig__createAckPacket(message_t *data, message_t *ack)
{
  CC2420XRadioP__Ieee154PacketLayer__createAckReply(data, ack);
}

# 80 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
inline static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__createAckPacket(message_t *data, message_t *ack){
#line 80
  CC2420XRadioP__SoftwareAckConfig__createAckPacket(data, ack);
#line 80
}
#line 80
# 110 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline bool CC2420XRadioP__SoftwareAckConfig__requiresAckReply(message_t *msg)
{
  return CC2420XRadioP__Ieee154PacketLayer__requiresAckReply(msg);
}

# 75 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
inline static bool /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__requiresAckReply(message_t *msg){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = CC2420XRadioP__SoftwareAckConfig__requiresAckReply(msg);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 50 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
inline static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__AckReceivedFlag__set(message_t *msg){
#line 50
  /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__PacketFlag__set(0U, msg);
#line 50
}
#line 50
# 65 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void )
{
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
}

# 73 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__stop(void ){
#line 73
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop();
#line 73
}
#line 73
# 108 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
static inline void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__cancel(uint8_t id)
{
  for (; 0; ) ;
  for (; 0; ) ;

  /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__stop();
  /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__state = /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__STATE_READY;
}

# 55 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
inline static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioAlarm__cancel(void ){
#line 55
  /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__cancel(2U);
#line 55
}
#line 55
# 124 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__verifyAckReply(message_t *data, message_t *ack)
{
  ieee154_simple_header_t *header = /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(ack);

  return __nesc_ntoh_leuint8(header->dsn.nxdata) == __nesc_ntoh_leuint8(/*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(data)->dsn.nxdata)
   && (__nesc_ntoh_leuint16(header->fcf.nxdata) & /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_ACK_FRAME_MASK) == /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_ACK_FRAME_VALUE;
}

# 94 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static bool CC2420XRadioP__Ieee154PacketLayer__verifyAckReply(message_t *data, message_t *ack){
#line 94
  unsigned char __nesc_result;
#line 94

#line 94
  __nesc_result = /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__verifyAckReply(data, ack);
#line 94

#line 94
  return __nesc_result;
#line 94
}
#line 94
# 100 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline bool CC2420XRadioP__SoftwareAckConfig__verifyAckPacket(message_t *data, message_t *ack)
{
  return CC2420XRadioP__Ieee154PacketLayer__verifyAckReply(data, ack);
}

# 69 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
inline static bool /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__verifyAckPacket(message_t *data, message_t *ack){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = CC2420XRadioP__SoftwareAckConfig__verifyAckPacket(data, ack);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 75 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static bool CC2420XRadioP__Ieee154PacketLayer__isAckFrame(message_t *msg){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isAckFrame(msg);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 95 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline bool CC2420XRadioP__SoftwareAckConfig__isAckPacket(message_t *msg)
{
  return CC2420XRadioP__Ieee154PacketLayer__isAckFrame(msg);
}

# 62 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
inline static bool /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__isAckPacket(message_t *msg){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = CC2420XRadioP__SoftwareAckConfig__isAckPacket(msg);
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 147 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckLayerC.nc"
static inline message_t */*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubReceive__receive(message_t *msg)
{
  for (; 0; ) ;

  if (/*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__isAckPacket(msg)) 
    {
      if (/*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__state == /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__STATE_ACK_WAIT && /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__verifyAckPacket(/*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__txMsg, msg)) 
        {
          /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioAlarm__cancel();
          /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__AckReceivedFlag__set(/*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__txMsg);

          /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__state = /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__STATE_READY;
          /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioSend__sendDone(SUCCESS);
        }

      return msg;
    }

  if (/*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__state == /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__STATE_READY && /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__requiresAckReply(msg)) 
    {
      /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__createAckPacket(msg, &/*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__ackMsg);


      if (/*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubSend__send(&/*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__ackMsg) == SUCCESS) {
        /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__state = /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__STATE_ACK_SEND;
        }
      else {
#line 173
        for (; 0; ) ;
        }
    }
  return /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioReceive__receive(msg);
}

# 53 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
inline static message_t *CC2420XDriverLayerP__RadioReceive__receive(message_t *msg){
#line 53
  nx_struct message_t *__nesc_result;
#line 53

#line 53
  __nesc_result = /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubReceive__receive(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 136 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TimeStampingLayerP.nc"
static inline uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__metadataLength(message_t *msg)
{
  return /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__SubPacket__metadataLength(msg) + sizeof(timestamp_metadata_t );
}

#line 60
static inline timestamp_metadata_t */*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__getMeta(message_t *msg)
{
  return (void *)msg + sizeof(message_t ) - /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__metadataLength(msg);
}

# 50 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
inline static void /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__TimeStampFlag__set(message_t *msg){
#line 50
  /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__PacketFlag__set(1U, msg);
#line 50
}
#line 50
# 82 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TimeStampingLayerP.nc"
static inline void /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__PacketTimeStampRadio__set(message_t *msg, uint32_t value)
{
  /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__TimeStampFlag__set(msg);
  /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__getMeta(msg)->timestamp = value;
}

# 78 "/opt/tinyos-2.1.2/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420XDriverLayerP__PacketTimeStamp__set(message_t * msg, CC2420XDriverLayerP__PacketTimeStamp__size_type value){
#line 78
  /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__PacketTimeStampRadio__set(msg, value);
#line 78
}
#line 78
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 53 "/opt/tinyos-2.1.2/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get(void )
{
  return /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__get();
}

# 61 "/opt/tinyos-2.1.2/tos/lib/timer/LocalTime.nc"
inline static uint32_t CC2420XDriverLayerP__LocalTime__get(void ){
#line 61
  unsigned long __nesc_result;
#line 61

#line 61
  __nesc_result = /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get();
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 65 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
inline static tradio_size CC2420XDriverLayerP__RadioAlarm__getNow(void ){
#line 65
  unsigned int __nesc_result;
#line 65

#line 65
  __nesc_result = /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__getNow(3U);
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 1307 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static inline void CC2420XDriverLayerP__PacketLinkQuality__set(message_t *msg, uint8_t value)
{
  CC2420XDriverLayerP__getMeta(msg)->lqi = value;
}

# 50 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
inline static void CC2420XDriverLayerP__RSSIFlag__set(message_t *msg){
#line 50
  /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__PacketFlag__set(3U, msg);
#line 50
}
#line 50





inline static void CC2420XDriverLayerP__TransmitPowerFlag__clear(message_t *msg){
#line 55
  /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__PacketFlag__clear(2U, msg);
#line 55
}
#line 55
# 1257 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static inline void CC2420XDriverLayerP__PacketRSSI__set(message_t *msg, uint8_t value)
{

  CC2420XDriverLayerP__TransmitPowerFlag__clear(msg);

  CC2420XDriverLayerP__RSSIFlag__set(msg);
  CC2420XDriverLayerP__getMeta(msg)->rssi = value;
}

# 312 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MessageBufferLayerP.nc"
static inline bool /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioReceive__header(message_t *msg)
{
  bool notFull;


  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 317
    notFull = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueueSize < /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RECEIVE_QUEUE_SIZE;
#line 317
    __nesc_atomic_end(__nesc_atomic); }

  return notFull;
}

# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
inline static bool /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__RadioReceive__header(message_t *msg){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioReceive__header(msg);
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 85 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/UniqueLayerP.nc"
static inline bool /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__SubReceive__header(message_t *msg)
{

  return /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__RadioReceive__header(msg);
}

# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
inline static bool /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioReceive__header(message_t *msg){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__SubReceive__header(msg);
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 163 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/RandomCollisionLayerP.nc"
static inline bool /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubReceive__header(message_t *msg)
{
  return /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioReceive__header(msg);
}

# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
inline static bool /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioReceive__header(message_t *msg){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__SubReceive__header(msg);
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 134 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckLayerC.nc"
static inline bool /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubReceive__header(message_t *msg)
{

  if (/*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__isAckPacket(msg)) {
    return /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__state == /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__STATE_ACK_WAIT;
    }




  return /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioReceive__header(msg);
}

# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
inline static bool CC2420XDriverLayerP__RadioReceive__header(message_t *msg){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubReceive__header(msg);
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 207 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static __inline cc2420X_status_t CC2420XDriverLayerP__getStatus(void )
#line 207
{
  return CC2420XDriverLayerP__strobe(CC2420X_SNOP);
}

# 66 "/opt/tinyos-2.1.2/tos/lib/timer/BusyWait.nc"
inline static void CC2420XDriverLayerP__BusyWait__wait(CC2420XDriverLayerP__BusyWait__size_type dt){
#line 66
  /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(dt);
#line 66
}
#line 66
# 776 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static __inline void CC2420XDriverLayerP__recover(void )
#line 776
{
  cc2420X_status_t status;

  CC2420XDriverLayerP__SfdCapture__disable();


  CC2420XDriverLayerP__resetRadio();

  for (; 0; ) ;


  CC2420XDriverLayerP__strobe(CC2420X_SXOSCON);


  CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_PD_2_IDLE;

  CC2420XDriverLayerP__BusyWait__wait(PD_2_IDLE_TIME);


  status = CC2420XDriverLayerP__getStatus();
  for (; 0; ) ;
  for (; 0; ) ;
  for (; 0; ) ;
  for (; 0; ) ;
  for (; 0; ) ;
  for (; 0; ) ;


  CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_IDLE;


  CC2420XDriverLayerP__setChannel();


  CC2420XDriverLayerP__strobe(CC2420X_SRXON);
  CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_IDLE_2_RX_ON;

  CC2420XDriverLayerP__SfdCapture__captureRisingEdge();


  CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_RX_ON;
}

# 59 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )32U & (0x01 << 3);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw() != 0;
}

# 73 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420XC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420XC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420XC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get();
}

# 43 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static bool CC2420XDriverLayerP__FIFO__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420XC.FIFOM*/Msp430GpioC__5__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 59 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )32U & (0x01 << 0);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw() != 0;
}

# 73 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get();
}

# 43 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static bool CC2420XDriverLayerP__FIFOP__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 319 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static __inline void CC2420XDriverLayerP__readCrcOkAndLqiFromRxFifo(uint8_t *crcOkAndLqiPtr)
{

  for (; 0; ) ;

  *crcOkAndLqiPtr = CC2420XDriverLayerP__FastSpiByte__splitRead();


  CC2420XDriverLayerP__CSN__set();
}

#line 256
static __inline uint8_t CC2420XDriverLayerP__waitForRxFifo(void )
#line 256
{



  uint16_t timeout = CC2420XDriverLayerP__RadioAlarm__getNow() + 4 * CC2420X_SYMBOL_TIME;

  while (CC2420XDriverLayerP__FIFO__get() == 0 && timeout - CC2420XDriverLayerP__RadioAlarm__getNow() < 0x7fff) ;
  return CC2420XDriverLayerP__FIFO__get();
}

#line 309
static __inline void CC2420XDriverLayerP__readRssiFromRxFifo(uint8_t *rssiPtr)
{

  for (; 0; ) ;

  *rssiPtr = CC2420XDriverLayerP__FastSpiByte__splitRead();
  CC2420XDriverLayerP__waitForRxFifo();
  CC2420XDriverLayerP__FastSpiByte__splitWrite(0);
}

#line 294
static __inline void CC2420XDriverLayerP__readPayloadFromRxFifo(uint8_t *data, uint8_t length)
{
  uint8_t idx;


  for (; 0; ) ;


  for (idx = 0; idx < length; idx++) {
      data[idx] = CC2420XDriverLayerP__FastSpiByte__splitRead();
      CC2420XDriverLayerP__waitForRxFifo();
      CC2420XDriverLayerP__FastSpiByte__splitWrite(0);
    }
}

#line 85
static inline cc2420x_header_t *CC2420XDriverLayerP__getHeader(message_t *msg)
{
  return (void *)msg + CC2420XDriverLayerP__Config__headerLength(msg);
}

# 67 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline uint8_t CC2420XRadioP__CC2420XDriverConfig__maxPayloadLength(void )
{
  return sizeof(cc2420xpacket_header_t ) + 28;
}

# 35 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverConfig.nc"
inline static uint8_t CC2420XDriverLayerP__Config__maxPayloadLength(void ){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = CC2420XRadioP__CC2420XDriverConfig__maxPayloadLength();
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 1200 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static inline uint8_t CC2420XDriverLayerP__RadioPacket__maxPayloadLength(void )
{
  for (; 0; ) ;

  return CC2420XDriverLayerP__Config__maxPayloadLength() - sizeof(cc2420x_header_t );
}

#line 249
static __inline uint8_t CC2420XDriverLayerP__waitForRxFifoNoTimeout(void )
#line 249
{

  while (CC2420XDriverLayerP__FIFO__get() == 0) ;

  return CC2420XDriverLayerP__FIFO__get();
}











static __inline cc2420X_status_t CC2420XDriverLayerP__readLengthFromRxFifo(uint8_t *lengthPtr)
{
  cc2420X_status_t status;

  for (; 0; ) ;
  for (; 0; ) ;


  CC2420XDriverLayerP__CSN__set();
  CC2420XDriverLayerP__CSN__clr();


  CC2420XDriverLayerP__waitForRxFifoNoTimeout();


  CC2420XDriverLayerP__FastSpiByte__splitWrite(CC2420X_CMD_REGISTER_READ | CC2420X_RXFIFO);
  status.value = CC2420XDriverLayerP__FastSpiByte__splitRead();
  CC2420XDriverLayerP__FastSpiByte__splitWrite(0);

  *lengthPtr = CC2420XDriverLayerP__FastSpiByte__splitRead();



  CC2420XDriverLayerP__waitForRxFifo();
  CC2420XDriverLayerP__FastSpiByte__splitWrite(0);
  return status;
}

#line 90
static inline void *CC2420XDriverLayerP__getPayload(message_t *msg)
{
  return (void *)msg;
}

#line 819
static __inline void CC2420XDriverLayerP__downloadMessage(void )
{
  uint8_t length;
  uint16_t crc = 1;
  uint8_t *data;
  uint8_t rssi;
  uint8_t crc_ok_lqi;
  uint16_t sfdTime;


  CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_RX_DOWNLOAD;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 831
    sfdTime = CC2420XDriverLayerP__capturedTime;
#line 831
    __nesc_atomic_end(__nesc_atomic); }


  data = CC2420XDriverLayerP__getPayload(CC2420XDriverLayerP__rxMsg) + sizeof(cc2420x_header_t );


  CC2420XDriverLayerP__readLengthFromRxFifo(&length);


  if (length == 0) {

      CC2420XDriverLayerP__CSN__set();

      for (; 0; ) ;
      for (; 0; ) ;

      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_RX_ON;
      CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_NONE;
      CC2420XDriverLayerP__SfdCapture__captureRisingEdge();
      return;
    }

  if (length == 1) {

      CC2420XDriverLayerP__readCrcOkAndLqiFromRxFifo(&crc_ok_lqi);

      for (; 0; ) ;
      for (; 0; ) ;

      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_RX_ON;
      CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_NONE;
      CC2420XDriverLayerP__SfdCapture__captureRisingEdge();
      return;
    }

  if (length == 2) {

      {
        CC2420XDriverLayerP__readRssiFromRxFifo(&rssi);
        CC2420XDriverLayerP__readCrcOkAndLqiFromRxFifo(&crc_ok_lqi);
      }

      for (; 0; ) ;
      for (; 0; ) ;

      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_RX_ON;
      CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_NONE;
      CC2420XDriverLayerP__SfdCapture__captureRisingEdge();
      return;
    }


  if (length > 127) {

      CC2420XDriverLayerP__recover();

      for (; 0; ) ;
      for (; 0; ) ;

      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_RX_ON;
      CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_NONE;
      CC2420XDriverLayerP__SfdCapture__captureRisingEdge();
      return;
    }

  if (length > CC2420XDriverLayerP__RadioPacket__maxPayloadLength() + 2) 
    {
      while (length-- > 2) {
          CC2420XDriverLayerP__readPayloadFromRxFifo(data, 1);
        }

      {
        CC2420XDriverLayerP__readRssiFromRxFifo(&rssi);
        CC2420XDriverLayerP__readCrcOkAndLqiFromRxFifo(&crc_ok_lqi);
      }

      for (; 0; ) ;

      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_RX_ON;
      CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_NONE;
      CC2420XDriverLayerP__SfdCapture__captureRisingEdge();
      return;
    }


  for (; 0; ) ;

  __nesc_hton_leuint8(CC2420XDriverLayerP__getHeader(CC2420XDriverLayerP__rxMsg)->length.nxdata, length);


  length -= 2;

  {

    CC2420XDriverLayerP__readPayloadFromRxFifo(data, length);


    CC2420XDriverLayerP__readRssiFromRxFifo(&rssi);
    CC2420XDriverLayerP__readCrcOkAndLqiFromRxFifo(&crc_ok_lqi);
  }



  if (CC2420XDriverLayerP__FIFOP__get() == 1 || CC2420XDriverLayerP__FIFO__get() == 1) {
      CC2420XDriverLayerP__recover();
      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_RX_ON;
      CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_NONE;
      CC2420XDriverLayerP__SfdCapture__captureRisingEdge();
      return;
    }

  CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_RX_ON;
  CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_NONE;


  CC2420XDriverLayerP__SfdCapture__captureRisingEdge();

  if (CC2420XDriverLayerP__RadioReceive__header(CC2420XDriverLayerP__rxMsg)) 
    {

      CC2420XDriverLayerP__PacketRSSI__set(CC2420XDriverLayerP__rxMsg, rssi);
      CC2420XDriverLayerP__PacketLinkQuality__set(CC2420XDriverLayerP__rxMsg, crc_ok_lqi & 0x7f);
      crc = crc_ok_lqi > 0x7f ? 0 : 1;
    }


  if (crc == 0) {
      uint32_t time32;
      uint16_t time;

#line 960
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 960
        {
          time = CC2420XDriverLayerP__RadioAlarm__getNow();
          time32 = CC2420XDriverLayerP__LocalTime__get();
        }
#line 963
        __nesc_atomic_end(__nesc_atomic); }


      time -= sfdTime;
      time32 -= time;

      CC2420XDriverLayerP__PacketTimeStamp__set(CC2420XDriverLayerP__rxMsg, time32);
#line 984
      CC2420XDriverLayerP__rxMsg = CC2420XDriverLayerP__RadioReceive__receive(CC2420XDriverLayerP__rxMsg);
    }
}

# 56 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
inline static void CC2420XDriverLayerP__RadioSend__sendDone(error_t error){
#line 56
  /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubSend__sendDone(error);
#line 56
}
#line 56
# 1035 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static __inline void CC2420XDriverLayerP__serviceRadio(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 1037
    if (CC2420XDriverLayerP__isSpiAcquired()) 
      {
        CC2420XDriverLayerP__radioIrq = FALSE;

        if (CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_RX_ON && CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_NONE) 
          {

            CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_DOWNLOAD;
          }
        else {
#line 1046
          if ((CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_TX_ON || CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_BUSY_TX_2_RX_ON) && CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_TRANSMIT) 
            {
              cc2420X_status_t status;


              CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_RX_ON;
              CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_NONE;
#line 1070
              CC2420XDriverLayerP__SfdCapture__captureRisingEdge();


              status = CC2420XDriverLayerP__getStatus();

              if (status.tx_underflow == 1) {

                  CC2420XDriverLayerP__strobe(CC2420X_SFLUSHTX);
                  CC2420XDriverLayerP__RadioSend__sendDone(FAIL);
                }
              else 
#line 1079
                {
                  CC2420XDriverLayerP__RadioSend__sendDone(SUCCESS);
                }
            }
          else {


            for (; 0; ) ;
            }
          }
      }
#line 1089
    __nesc_atomic_end(__nesc_atomic); }
}

#line 1107
static inline void CC2420XDriverLayerP__Tasklet__run(void )
{
#line 1131
  if (CC2420XDriverLayerP__radioIrq) {
    CC2420XDriverLayerP__serviceRadio();
    }
  if (CC2420XDriverLayerP__cmd != CC2420XDriverLayerP__CMD_NONE) 
    {
      if (CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_DOWNLOAD && CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_RX_ON) {
        CC2420XDriverLayerP__downloadMessage();
        }
      else {
#line 1138
        if (CC2420XDriverLayerP__CMD_TURNOFF <= CC2420XDriverLayerP__cmd && CC2420XDriverLayerP__cmd <= CC2420XDriverLayerP__CMD_TURNON) {
          CC2420XDriverLayerP__changeState();
          }
        else {
#line 1140
          if (CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_CHANNEL) {
            CC2420XDriverLayerP__changeChannel();
            }
          }
        }
#line 1143
      if (CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_SIGNAL_DONE) 
        {
          CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_NONE;
          CC2420XDriverLayerP__RadioState__done();
        }
    }

  if (CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_NONE && CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_RX_ON && !CC2420XDriverLayerP__radioIrq) {
    CC2420XDriverLayerP__RadioSend__ready();
    }
  if (CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_NONE) {
    CC2420XDriverLayerP__releaseSpi__postTask();
    }
}

# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
inline static void TaskletC__Tasklet__run(void ){
#line 48
  CC2420XDriverLayerP__Tasklet__run();
#line 48
  /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Tasklet__run();
#line 48
  /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Tasklet__run();
#line 48
}
#line 48
# 177 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id)
#line 177
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 178
    {
      if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id && /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) {
          unsigned char __nesc_temp = 
#line 179
          TRUE;

          {
#line 179
            __nesc_atomic_end(__nesc_atomic); 
#line 179
            return __nesc_temp;
          }
        }
      else 
#line 180
        {
          unsigned char __nesc_temp = 
#line 180
          FALSE;

          {
#line 180
            __nesc_atomic_end(__nesc_atomic); 
#line 180
            return __nesc_temp;
          }
        }
    }
#line 183
    __nesc_atomic_end(__nesc_atomic); }
}

# 172 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(uint8_t id)
#line 172
{
#line 172
  return FALSE;
}

# 128 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(uint8_t arg_0x2b395af86898){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  switch (arg_0x2b395af86898) {
#line 128
    case /*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 128
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(/*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 128
      break;
#line 128
    default:
#line 128
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(arg_0x2b395af86898);
#line 128
      break;
#line 128
    }
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
# 112 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(uint8_t id)
#line 112
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(id);
}

# 128 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static bool CC2420XDriverLayerP__SpiResource__isOwner(void ){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(/*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
# 176 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id)
#line 176
{
  return &msp430_spi_default_config;
}

# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
inline static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(uint8_t arg_0x2b395af85b40){
#line 39
  union __nesc_unnamed4287 *__nesc_result;
#line 39

#line 39
    __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(arg_0x2b395af85b40);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 168 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config){
#line 168
  HplMsp430Usart0P__Usart__setModeSpi(config);
#line 168
}
#line 168
# 120 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(uint8_t id)
#line 120
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(id));
}

# 216 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 216
{
}

# 59 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x2b395b0da6e0){
#line 59
  switch (arg_0x2b395b0da6e0) {
#line 59
    case /*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 59
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(/*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 59
      break;
#line 59
    default:
#line 59
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(arg_0x2b395b0da6e0);
#line 59
      break;
#line 59
    }
#line 59
}
#line 59
# 213 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void )
#line 213
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
}

# 81 "/opt/tinyos-2.1.2/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void ){
#line 81
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested();
#line 81
}
#line 81
# 206 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id)
#line 206
{
}

# 61 "/opt/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(uint8_t arg_0x2b395b0dc4b8){
#line 61
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(arg_0x2b395b0dc4b8);
#line 61
}
#line 61
# 93 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id)
#line 93
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 95
    {
      if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) {
          /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING;
          /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = id;
        }
      else {
          unsigned char __nesc_temp = 
#line 100
          FAIL;

          {
#line 100
            __nesc_atomic_end(__nesc_atomic); 
#line 100
            return __nesc_temp;
          }
        }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
#line 102
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested();
  if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
      return SUCCESS;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 107
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
#line 107
    __nesc_atomic_end(__nesc_atomic); }
  return FAIL;
}

# 174 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(uint8_t id)
#line 174
{
#line 174
  return FAIL;
}

# 97 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(uint8_t arg_0x2b395af86898){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  switch (arg_0x2b395af86898) {
#line 97
    case /*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 97
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(/*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 97
      break;
#line 97
    default:
#line 97
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(arg_0x2b395af86898);
#line 97
      break;
#line 97
    }
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 104 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(uint8_t id)
#line 104
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(id);
}

# 97 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420XDriverLayerP__SpiResource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(/*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 151 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__resetUsart(bool reset)
#line 151
{
  if (reset) {
      U0CTL = 0x01;
    }
  else {
      U0CTL &= ~0x01;
    }
}

# 97 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset){
#line 97
  HplMsp430Usart0P__Usart__resetUsart(reset);
#line 97
}
#line 97
# 59 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static inline void HplMsp430I2C0P__HplI2C__clearModeI2C(void )
#line 59
{
  /* atomic removed: atomic calls only */
#line 60
  {
    HplMsp430I2C0P__U0CTL &= ~((0x20 | 0x04) | 0x01);
    HplMsp430I2C0P__HplUsart__resetUsart(TRUE);
  }
}

# 7 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static void HplMsp430Usart0P__HplI2C__clearModeI2C(void ){
#line 7
  HplMsp430I2C0P__HplI2C__clearModeI2C();
#line 7
}
#line 7
# 67 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 5);
}

# 99 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__URXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 4);
}

# 99 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UTXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc();
#line 99
}
#line 99
# 207 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__disableUart(void )
#line 207
{
  /* atomic removed: atomic calls only */
#line 208
  {
    HplMsp430Usart0P__ME1 &= ~(0x80 | 0x40);
    HplMsp430Usart0P__UTXD__selectIOFunc();
    HplMsp430Usart0P__URXD__selectIOFunc();
  }
}

#line 143
static inline void HplMsp430Usart0P__Usart__setUmctl(uint8_t control)
#line 143
{
  U0MCTL = control;
}

#line 132
static inline void HplMsp430Usart0P__Usart__setUbr(uint16_t control)
#line 132
{
  /* atomic removed: atomic calls only */
#line 133
  {
    U0BR0 = control & 0x00FF;
    U0BR1 = (control >> 8) & 0x00FF;
  }
}

#line 256
static inline void HplMsp430Usart0P__configSpi(msp430_spi_union_config_t *config)
#line 256
{

  U0CTL = (config->spiRegisters.uctl | 0x04) | 0x01;
  HplMsp430Usart0P__U0TCTL = config->spiRegisters.utctl;

  HplMsp430Usart0P__Usart__setUbr(config->spiRegisters.ubr);
  HplMsp430Usart0P__Usart__setUmctl(0x00);
}

# 65 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 3;
}

# 92 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc();
#line 92
}
#line 92
# 65 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 2;
}

# 92 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc();
#line 92
}
#line 92
# 65 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 1;
}

# 92 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc();
#line 92
}
#line 92
# 238 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__enableSpi(void )
#line 238
{
  /* atomic removed: atomic calls only */
#line 239
  {
    HplMsp430Usart0P__SIMO__selectModuleFunc();
    HplMsp430Usart0P__SOMI__selectModuleFunc();
    HplMsp430Usart0P__UCLK__selectModuleFunc();
  }
  HplMsp430Usart0P__ME1 |= 0x40;
}

#line 345
static inline void HplMsp430Usart0P__Usart__clrIntr(void )
#line 345
{
  HplMsp430Usart0P__IFG1 &= ~(0x80 | 0x40);
}









static inline void HplMsp430Usart0P__Usart__disableIntr(void )
#line 357
{
  HplMsp430Usart0P__IE1 &= ~(0x80 | 0x40);
}

# 204 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(uint8_t id)
#line 204
{
}

# 53 "/opt/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(uint8_t arg_0x2b395b0dc4b8){
#line 53
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(arg_0x2b395b0dc4b8);
#line 53
}
#line 53
# 64 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 64
{
  /* atomic removed: atomic calls only */
#line 65
  {
    unsigned char __nesc_temp = 
#line 65
    /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[id] != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY || /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail == id;

#line 65
    return __nesc_temp;
  }
}

#line 82
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id)
#line 82
{
  /* atomic removed: atomic calls only */
#line 83
  {
    if (!/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(id)) {
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = id;
          }
        else {
#line 88
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail] = id;
          }
#line 89
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = id;
        {
          unsigned char __nesc_temp = 
#line 90
          SUCCESS;

#line 90
          return __nesc_temp;
        }
      }
#line 92
    {
      unsigned char __nesc_temp = 
#line 92
      EBUSY;

#line 92
      return __nesc_temp;
    }
  }
}

# 79 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
inline static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(resource_client_id_t id){
#line 79
  unsigned char __nesc_result;
#line 79

#line 79
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(id);
#line 79

#line 79
  return __nesc_result;
#line 79
}
#line 79
# 210 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void )
#line 210
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
}

# 73 "/opt/tinyos-2.1.2/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested(void ){
#line 73
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested();
#line 73
}
#line 73
# 135 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void )
{
  * (volatile uint16_t * )388U &= ~0x0010;
}

# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents();
#line 58
}
#line 58
# 65 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )31U |= 0x01 << 1;
}

# 92 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__GeneralIO__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc();
#line 92
}
#line 92
# 57 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4349 {
#line 57
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t f;
#line 57
    uint16_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

#line 72
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(uint8_t l_cm)
{
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x = { 
  .cm = l_cm & 0x03, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 1, 
  .scs = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(x);
}

#line 110
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm)
{
  * (volatile uint16_t * )388U = /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(cm);
}

# 55 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm){
#line 55
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(cm);
#line 55
}
#line 55
# 130 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void )
{
  * (volatile uint16_t * )388U |= 0x0010;
}

# 57 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents();
#line 57
}
#line 57
# 198 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__requiresAckWait(message_t *msg)
{
  return /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getAckRequired(msg)
   && /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isDataFrame(msg)
   && /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDestAddr(msg) != 0xFFFF;
}

# 162 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static bool CC2420XRadioP__Ieee154PacketLayer__requiresAckWait(message_t *msg){
#line 162
  unsigned char __nesc_result;
#line 162

#line 162
  __nesc_result = /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__requiresAckWait(msg);
#line 162

#line 162
  return __nesc_result;
#line 162
}
#line 162
# 90 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline bool CC2420XRadioP__SoftwareAckConfig__requiresAckWait(message_t *msg)
{
  return CC2420XRadioP__Ieee154PacketLayer__requiresAckWait(msg);
}

# 55 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
inline static bool /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__requiresAckWait(message_t *msg){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420XRadioP__SoftwareAckConfig__requiresAckWait(msg);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 45 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
inline static bool /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioAlarm__isFree(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__isFree(2U);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 60 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__start(uint16_t dt)
{
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(/*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__getNow(), dt);
}

# 66 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__start(/*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__size_type dt){
#line 66
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__start(dt);
#line 66
}
#line 66
# 99 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
static inline void /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__wait(uint8_t id, tradio_size timeout)
{
  for (; 0; ) ;

  /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__alarm = id;
  /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__state = /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__STATE_WAIT;
  /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__start(timeout);
}

# 50 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
inline static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioAlarm__wait(tradio_size timeout){
#line 50
  /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__wait(2U, timeout);
#line 50
}
#line 50
# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 165 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get() + x;
}

# 43 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 155 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

# 41 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(time);
#line 41
}
#line 41
# 95 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

# 44 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 130 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

# 57 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents();
#line 57
}
#line 57
# 124 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline uint16_t CC2420XRadioP__SoftwareAckConfig__getAckTimeout(void )
{
  return (uint16_t )(800 * 1);
}

# 43 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
inline static uint16_t /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__getAckTimeout(void ){
#line 43
  unsigned int __nesc_result;
#line 43

#line 43
  __nesc_result = CC2420XRadioP__SoftwareAckConfig__getAckTimeout();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 67 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )31U &= ~(0x01 << 1);
}

# 99 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__GeneralIO__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc();
#line 99
}
#line 99
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
static __inline uint32_t Msp430HybridAlarmCounterP__now2ghz(void )
#line 86
{
  uint16_t t32khz;
#line 87
  uint16_t tMicro;
  uint32_t t2ghz;

  Msp430HybridAlarmCounterP__now(&t32khz, &tMicro, &t2ghz);

  return t2ghz;
}







static inline uint32_t Msp430HybridAlarmCounterP__Counter2ghz__get(void )
#line 101
{
  return Msp430HybridAlarmCounterP__now2ghz();
}

# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__size_type /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = Msp430HybridAlarmCounterP__Counter2ghz__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64







inline static bool Msp430HybridAlarmCounterP__Counter32khz__isOverflowPending(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 110 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
static inline bool Msp430HybridAlarmCounterP__Counter2ghz__isOverflowPending(void )
#line 110
{
  return Msp430HybridAlarmCounterP__Counter32khz__isOverflowPending();
}

# 71 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static bool /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__isOverflowPending(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = Msp430HybridAlarmCounterP__Counter2ghz__isOverflowPending();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 40 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
inline static bool CC2420XDriverLayerP__TransmitPowerFlag__get(message_t *msg){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__PacketFlag__get(2U, msg);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 1219 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static inline bool CC2420XDriverLayerP__PacketTransmitPower__isSet(message_t *msg)
{
  return CC2420XDriverLayerP__TransmitPowerFlag__get(msg);
}

static inline uint8_t CC2420XDriverLayerP__PacketTransmitPower__get(message_t *msg)
{
  return CC2420XDriverLayerP__getMeta(msg)->power;
}

# 63 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static bool CC2420XRadioP__Ieee154PacketLayer__isDataFrame(message_t *msg){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isDataFrame(msg);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 83 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline bool CC2420XRadioP__CC2420XDriverConfig__requiresRssiCca(message_t *msg)
{
  return CC2420XRadioP__Ieee154PacketLayer__isDataFrame(msg);
}

# 54 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverConfig.nc"
inline static bool CC2420XDriverLayerP__Config__requiresRssiCca(message_t *msg){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = CC2420XRadioP__CC2420XDriverConfig__requiresRssiCca(msg);
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 59 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )32U & (0x01 << 4);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw() != 0;
}

# 73 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420XC.CCAM*/Msp430GpioC__3__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420XC.CCAM*/Msp430GpioC__3__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420XC.CCAM*/Msp430GpioC__3__HplGeneralIO__get();
}

# 43 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static bool CC2420XDriverLayerP__CCA__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420XC.CCAM*/Msp430GpioC__3__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 77 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline uint8_t CC2420XRadioP__CC2420XDriverConfig__headerPreloadLength(void )
{

  return 7;
}

# 48 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverConfig.nc"
inline static uint8_t CC2420XDriverLayerP__Config__headerPreloadLength(void ){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = CC2420XRadioP__CC2420XDriverConfig__headerPreloadLength();
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 230 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static __inline cc2420X_status_t CC2420XDriverLayerP__writeTxFifo(uint8_t *data, uint8_t length)
{
  cc2420X_status_t status;
  uint8_t idx;

  for (; 0; ) ;

  CC2420XDriverLayerP__CSN__set();
  CC2420XDriverLayerP__CSN__clr();

  CC2420XDriverLayerP__FastSpiByte__splitWrite(CC2420X_CMD_REGISTER_WRITE | CC2420X_TXFIFO);
  for (idx = 0; idx < length; idx++) 
    CC2420XDriverLayerP__FastSpiByte__splitReadWrite(data[idx]);
  status.value = CC2420XDriverLayerP__FastSpiByte__splitRead();

  CC2420XDriverLayerP__CSN__set();
  return status;
}

# 59 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )28U & (0x01 << 1);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw() != 0;
}

# 73 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420XC.SFDM*/Msp430GpioC__8__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420XC.SFDM*/Msp430GpioC__8__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420XC.SFDM*/Msp430GpioC__8__HplGeneralIO__get();
}

# 43 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static bool CC2420XDriverLayerP__SFD__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420XC.SFDM*/Msp430GpioC__8__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 65 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline error_t /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captureFallingEdge(void )
#line 65
{
  return /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__enableCapture(MSP430TIMER_CM_FALLING);
}

# 54 "/opt/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420XDriverLayerP__SfdCapture__captureFallingEdge(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captureFallingEdge();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 40 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
inline static bool CC2420XDriverLayerP__TimeSyncFlag__get(message_t *msg){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__PacketFlag__get(4U, msg);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 1268 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static inline bool CC2420XDriverLayerP__PacketTimeSyncOffset__isSet(message_t *msg)
{
  return CC2420XDriverLayerP__TimeSyncFlag__get(msg);
}

static inline uint8_t CC2420XDriverLayerP__PacketTimeSyncOffset__get(message_t *msg)
{
  return CC2420XDriverLayerP__RadioPacket__headerLength(msg) + CC2420XDriverLayerP__RadioPacket__payloadLength(msg) - sizeof(timesync_absolute_t );
}

# 347 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint32_t __nesc_hton_uint32(void * target, uint32_t value)
#line 347
{
  uint8_t *base = target;

#line 349
  base[3] = value;
  base[2] = value >> 8;
  base[1] = value >> 16;
  base[0] = value >> 24;
  return value;
}

#line 372
static __inline  int32_t __nesc_hton_int32(void * target, int32_t value)
#line 372
{
#line 372
  __nesc_hton_uint32(target, value);
#line 372
  return value;
}

#line 340
static __inline  uint32_t __nesc_ntoh_uint32(const void * source)
#line 340
{
  const uint8_t *base = source;

#line 342
  return ((((uint32_t )base[0] << 24) | (
  (uint32_t )base[1] << 16)) | (
  (uint32_t )base[2] << 8)) | base[3];
}

# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__calcNextRandom__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__calcNextRandom);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 229 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline uint16_t CC2420XRadioP__RandomCollisionConfig__getMinimumBackoff(void )
{
  return (uint16_t )(320 * 1);
}

# 51 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/RandomCollisionConfig.nc"
inline static uint16_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__Config__getMinimumBackoff(void ){
#line 51
  unsigned int __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420XRadioP__RandomCollisionConfig__getMinimumBackoff();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 192 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void )
{
}

# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired();
#line 45
}
#line 45
# 150 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

# 994 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static inline void CC2420XDriverLayerP__SfdCapture__captured(uint16_t time)
{
#line 1016
  for (; 0; ) ;
  for (; 0; ) ;
  /* atomic removed: atomic calls only */
  CC2420XDriverLayerP__capturedTime = time;

  CC2420XDriverLayerP__radioIrq = TRUE;
  CC2420XDriverLayerP__SfdCapture__disable();



  CC2420XDriverLayerP__Tasklet__schedule();
}

# 61 "/opt/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
inline static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captured(uint16_t time){
#line 61
  CC2420XDriverLayerP__SfdCapture__captured(time);
#line 61
}
#line 61
# 175 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void )
{
  * (volatile uint16_t * )388U &= ~0x0002;
}

# 68 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430Capture__clearOverflow(void ){
#line 68
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow();
#line 68
}
#line 68
# 95 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )388U &= ~0x0001;
}

# 44 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 76 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time)
#line 76
{
  /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt();
  /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430Capture__clearOverflow();
  /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captured(time);
}

# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 86
  /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430Capture__captured(time);
#line 86
}
#line 86
# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4350 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(* (volatile uint16_t * )388U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired();
    }
}

# 191 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
static inline void Msp430HybridAlarmCounterP__Alarm2ghz__default__fired(void )
#line 191
{
}

# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void Msp430HybridAlarmCounterP__Alarm2ghz__fired(void ){
#line 78
  Msp430HybridAlarmCounterP__Alarm2ghz__default__fired();
#line 78
}
#line 78
# 176 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
static inline void Msp430HybridAlarmCounterP__AlarmMicro__fired(void )
#line 176
{

  Msp430HybridAlarmCounterP__Alarm2ghz__fired();
}

# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__fired(void ){
#line 78
  Msp430HybridAlarmCounterP__AlarmMicro__fired();
#line 78
}
#line 78
# 135 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void )
{
  * (volatile uint16_t * )390U &= ~0x0010;
}

# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents();
#line 58
}
#line 58
# 70 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__fired(void )
{
  /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__disableEvents();
  /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__fired();
}

# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 45
  /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__fired();
#line 45
}
#line 45
# 150 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n)
{
}

# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4351 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(* (volatile uint16_t * )390U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void )
{
}

# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired();
#line 45
}
#line 45
# 150 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n)
{
}

# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4352 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(* (volatile uint16_t * )392U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void )
{
}

# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired();
#line 45
}
#line 45
# 150 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n)
{
}

# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4353 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(* (volatile uint16_t * )394U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void )
{
}

# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired();
#line 45
}
#line 45
# 150 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n)
{
}

# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4354 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(* (volatile uint16_t * )396U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void )
{
}

# 45 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired();
#line 45
}
#line 45
# 150 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n)
{
}

# 86 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4355 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(* (volatile uint16_t * )398U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired();
    }
}

# 131 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 134
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(n >> 1);
}

# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB1__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired();
#line 39
}
#line 39
# 124 "/opt/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 57 "/opt/tinyos-2.1.2/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__init(void ){
#line 57
  SchedulerBasicP__Scheduler__init();
#line 57
}
#line 57
# 56 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 6;
}

# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set();
#line 48
}
#line 48
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set();
}

# 40 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__set(void ){
#line 40
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set();
#line 40
}
#line 40
# 56 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 5;
}

# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set();
#line 48
}
#line 48
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set();
}

# 40 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__set(void ){
#line 40
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set();
#line 40
}
#line 40
# 56 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 4;
}

# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set();
#line 48
}
#line 48
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set();
}

# 40 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__set(void ){
#line 40
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set();
#line 40
}
#line 40
# 63 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

# 85 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput();
#line 85
}
#line 85
# 54 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput();
}

# 46 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 5;
}

# 85 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput();
#line 85
}
#line 85
# 54 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput();
}

# 46 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

# 85 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput();
#line 85
}
#line 85
# 54 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput();
}

# 46 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput();
#line 46
}
#line 46
# 56 "/opt/tinyos-2.1.2/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 57
  {
    ;
    LedsP__Led0__makeOutput();
    LedsP__Led1__makeOutput();
    LedsP__Led2__makeOutput();
    LedsP__Led0__set();
    LedsP__Led1__set();
    LedsP__Led2__set();
  }
  return SUCCESS;
}

# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t PlatformP__LedsInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = LedsP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 36 "/opt/tinyos-2.1.2/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r |= 1 << 1;
}

#line 37
static inline  void TOSH_SET_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r |= 1 << 3;
}

#line 88
static inline  void TOSH_SET_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r |= 1 << 4;
}

#line 37
static inline  void TOSH_CLR_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r &= ~(1 << 3);
}

#line 88
static inline  void TOSH_CLR_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r &= ~(1 << 4);
}

# 11 "/opt/tinyos-2.1.2/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__TOSH_wait(void )
#line 11
{
  __nop();
#line 12
  __nop();
}

# 89 "/opt/tinyos-2.1.2/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_FLASH_HOLD_PIN()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001D");

#line 89
  r |= 1 << 7;
}

#line 88
static inline  void TOSH_MAKE_FLASH_CS_OUTPUT()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001E");

#line 88
  r |= 1 << 4;
}

#line 89
static inline  void TOSH_MAKE_FLASH_HOLD_OUTPUT()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001E");

#line 89
  r |= 1 << 7;
}

#line 37
static inline  void TOSH_MAKE_UCLK0_OUTPUT()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x001A");

#line 37
  r |= 1 << 3;
}

#line 36
static inline  void TOSH_MAKE_SIMO0_OUTPUT()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x001A");

#line 36
  r |= 1 << 1;
}

# 27 "/opt/tinyos-2.1.2/tos/platforms/telosb/MotePlatformC.nc"
static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void )
#line 27
{

  TOSH_MAKE_SIMO0_OUTPUT();
  TOSH_MAKE_UCLK0_OUTPUT();
  TOSH_MAKE_FLASH_HOLD_OUTPUT();
  TOSH_MAKE_FLASH_CS_OUTPUT();
  TOSH_SET_FLASH_HOLD_PIN();
  TOSH_SET_FLASH_CS_PIN();

  MotePlatformC__TOSH_wait();


  TOSH_CLR_FLASH_CS_PIN();
  TOSH_CLR_UCLK0_PIN();

  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);

  TOSH_SET_FLASH_CS_PIN();
  TOSH_SET_UCLK0_PIN();
  TOSH_SET_SIMO0_PIN();
}

#line 6
static __inline void MotePlatformC__uwait(uint16_t u)
#line 6
{
  uint16_t t0 = TAR;

#line 8
  while (TAR - t0 <= u) ;
}

#line 56
static inline error_t MotePlatformC__Init__init(void )
#line 56
{
  /* atomic removed: atomic calls only */

  {
    P1SEL = 0;
    P2SEL = 0;
    P3SEL = 0;
    P4SEL = 0;
    P5SEL = 0;
    P6SEL = 0;

    P1OUT = 0x00;
    P1DIR = 0xe0;

    P2OUT = 0x30;
    P2DIR = 0x7b;

    P3OUT = 0x00;
    P3DIR = 0xf1;

    P4OUT = 0xdd;
    P4DIR = 0xfd;

    P5OUT = 0xff;
    P5DIR = 0xff;

    P6OUT = 0x00;
    P6DIR = 0xff;

    P1IE = 0;
    P2IE = 0;






    MotePlatformC__uwait(1024 * 10);

    MotePlatformC__TOSH_FLASH_M25P_DP();
  }

  return SUCCESS;
}

# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = MotePlatformC__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 160 "/opt/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/Msp430ClockP.nc"
static inline void Msp430ClockP__startTimerB(void )
{

  Msp430ClockP__TBCTL = 0x0020 | (Msp430ClockP__TBCTL & ~(0x0020 | 0x0010));
}

#line 148
static inline void Msp430ClockP__startTimerA(void )
{

  Msp430ClockP__TACTL = 0x0020 | (Msp430ClockP__TACTL & ~(0x0020 | 0x0010));
}

#line 112
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP__TBCTL = (0x0200 | 0x0080) | 0x0002;
}

#line 142
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerB();
}

# 43 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerB(void ){
#line 43
  Msp430ClockP__Msp430ClockInit__default__initTimerB();
#line 43
}
#line 43
# 97 "/opt/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void )
{
  TAR = 0;









  Msp430ClockP__TACTL = 0x0100 | 0x0002;
}

#line 137
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerA();
}

# 42 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerA(void ){
#line 42
  Msp430ClockP__Msp430ClockInit__default__initTimerA();
#line 42
}
#line 42
# 76 "/opt/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void )
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0;


  Msp430ClockP__IE1 &= ~0x02;
}

#line 132
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitClocks();
}

# 41 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initClocks(void ){
#line 41
  Msp430ClockP__Msp430ClockInit__default__initClocks();
#line 41
}
#line 41
# 178 "/opt/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/Msp430ClockP.nc"
static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  Msp430ClockP__set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TACCR0 = TAR + Msp430ClockP__ACLK_CALIB_PERIOD;
      TACCTL0 &= ~0x0001;
      while ((TACCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TBR;
    }

  return dco_curr - dco_prev;
}




static inline void Msp430ClockP__busyCalibrateDco(void )
{

  int calib;
  int step;






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (Msp430ClockP__test_calib_busywait_delta(calib | step) <= Msp430ClockP__TARGET_DCO_DELTA) {
        calib |= step;
        }
    }

  if ((calib & 0x0e0) == 0x0e0) {
    calib &= ~0x01f;
    }
  Msp430ClockP__set_dco_calib(calib);
}

#line 64
static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void )
{



  Msp430ClockP__TACTL = 0x0100 | 0x0020;
  Msp430ClockP__TBCTL = 0x0200 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;
}

#line 127
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void )
{
  Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate();
}

# 40 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void ){
#line 40
  Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate();
#line 40
}
#line 40
# 226 "/opt/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/Msp430ClockP.nc"
static inline error_t Msp430ClockP__Init__init(void )
{

  Msp430ClockP__TACTL = 0x0004;
  Msp430ClockP__TAIV = 0;
  Msp430ClockP__TBCTL = 0x0004;
  Msp430ClockP__TBIV = 0;
  /* atomic removed: atomic calls only */

  {
    Msp430ClockP__Msp430ClockInit__setupDcoCalibrate();
    Msp430ClockP__busyCalibrateDco();
    Msp430ClockP__Msp430ClockInit__initClocks();
    Msp430ClockP__Msp430ClockInit__initTimerA();
    Msp430ClockP__Msp430ClockInit__initTimerB();
    Msp430ClockP__startTimerA();
    Msp430ClockP__startTimerB();
  }

  return SUCCESS;
}

# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteClockInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430ClockP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 10 "/opt/tinyos-2.1.2/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 10
{
  WDTCTL = 0x5A00 + 0x0080;
  PlatformP__MoteClockInit__init();
  PlatformP__MoteInit__init();
  PlatformP__LedsInit__init();
  return SUCCESS;
}

# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t RealMainP__PlatformInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = PlatformP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 36 "/opt/tinyos-2.1.2/tos/platforms/telosb/hardware.h"
static inline  void TOSH_CLR_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r &= ~(1 << 1);
}

# 65 "/opt/tinyos-2.1.2/tos/interfaces/Scheduler.nc"
inline static bool RealMainP__Scheduler__runNextTask(void ){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = SchedulerBasicP__Scheduler__runNextTask();
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 110 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
inline static void /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(message_t * msg, error_t error){
#line 110
  BlinkToRadioC__AMSend__sendDone(msg, error);
#line 110
}
#line 110
# 65 "/opt/tinyos-2.1.2/tos/system/AMQueueEntryP.nc"
static inline void /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err)
#line 65
{
  /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(m, err);
}

# 215 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err)
#line 215
{
}

# 100 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(uint8_t arg_0x2b395b469318, message_t * msg, error_t error){
#line 100
  switch (arg_0x2b395b469318) {
#line 100
    case 0U:
#line 100
      /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(msg, error);
#line 100
      break;
#line 100
    default:
#line 100
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(arg_0x2b395b469318, msg, error);
#line 100
      break;
#line 100
    }
#line 100
}
#line 100
# 126 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void )
#line 126
{
  uint8_t i;
#line 127
  uint8_t j;
#line 127
  uint8_t mask;
#line 127
  uint8_t last;
  message_t *msg;

#line 129
  for (i = 0; i < 1 / 8 + 1; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i]) {
          for (mask = 1, j = 0; j < 8; j++) {
              if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i] & mask) {
                  last = i * 8 + j;
                  msg = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i] &= ~mask;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, ECANCEL);
                }
              mask <<= 1;
            }
        }
    }
}

#line 169
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void )
#line 169
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current, /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg, FAIL);
}

# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 80 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(am_id_t arg_0x2b395b468488, am_addr_t addr, message_t * msg, uint8_t len){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMSend__send(arg_0x2b395b468488, addr, msg, len);
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__SubPacket__payloadLength(message_t *msg){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = CC2420XDriverLayerP__RadioPacket__payloadLength(msg);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 95 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MetadataFlagsLayerC.nc"
static inline uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__payloadLength(message_t *msg)
{
  return /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__SubPacket__payloadLength(msg);
}

# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__SubPacket__payloadLength(message_t *msg){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__payloadLength(msg);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 121 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TimeStampingLayerP.nc"
static inline uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__payloadLength(message_t *msg)
{
  return /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__SubPacket__payloadLength(msg);
}

# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__payloadLength(message_t *msg){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__payloadLength(msg);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 287 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline uint8_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__RadioPacket__payloadLength(message_t *msg)
{
  return /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__payloadLength(msg) - sizeof(ieee154_simple_header_t );
}

# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubPacket__payloadLength(message_t *msg){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__RadioPacket__payloadLength(msg);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 159 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TinyosNetworkLayerC.nc"
static inline uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosPacket__payloadLength(message_t *msg)
{
  return /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubPacket__payloadLength(msg) - /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__PAYLOAD_OFFSET;
}

# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubPacket__payloadLength(message_t *msg){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosPacket__payloadLength(msg);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 230 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
static inline uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__RadioPacket__payloadLength(message_t *msg)
{
  return /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubPacket__payloadLength(msg) - sizeof(activemessage_header_t );
}

#line 262
static inline uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Packet__payloadLength(message_t *msg)
{
  return /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__RadioPacket__payloadLength(msg);
}

# 78 "/opt/tinyos-2.1.2/tos/interfaces/Packet.nc"
inline static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(message_t * msg){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Packet__payloadLength(msg);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 141 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static uint16_t CC2420XRadioP__Ieee154PacketLayer__getDestAddr(message_t *msg){
#line 141
  unsigned int __nesc_result;
#line 141

#line 141
  __nesc_result = /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDestAddr(msg);
#line 141

#line 141
  return __nesc_result;
#line 141
}
#line 141
# 162 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline am_addr_t CC2420XRadioP__ActiveMessageConfig__destination(message_t *msg)
{
  return CC2420XRadioP__Ieee154PacketLayer__getDestAddr(msg);
}

# 40 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageConfig.nc"
inline static am_addr_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Config__destination(message_t *msg){
#line 40
  unsigned int __nesc_result;
#line 40

#line 40
  __nesc_result = CC2420XRadioP__ActiveMessageConfig__destination(msg);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 179 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
static __inline am_addr_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__destination(message_t *msg)
{
  return /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Config__destination(msg);
}

# 78 "/opt/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
inline static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(message_t * amsg){
#line 78
  unsigned int __nesc_result;
#line 78

#line 78
  __nesc_result = /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__destination(amsg);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 281 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_uint8(const void * source)
#line 281
{
  const uint8_t *base = source;

#line 283
  return base[0];
}

# 43 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubPacket__headerLength(message_t *msg){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__RadioPacket__headerLength(msg);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 154 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TinyosNetworkLayerC.nc"
static inline uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosPacket__headerLength(message_t *msg)
{
  return /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubPacket__headerLength(msg) + /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__PAYLOAD_OFFSET;
}

# 43 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubPacket__headerLength(message_t *msg){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosPacket__headerLength(msg);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 67 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
static inline activemessage_header_t */*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__getHeader(message_t *msg)
{
  return (void *)msg + /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubPacket__headerLength(msg);
}

#line 199
static __inline am_id_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__type(message_t *msg)
{
  return __nesc_ntoh_uint8(/*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__getHeader(msg)->type.nxdata);
}

# 147 "/opt/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
inline static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(message_t * amsg){
#line 147
  unsigned char __nesc_result;
#line 147

#line 147
  __nesc_result = /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__type(amsg);
#line 147

#line 147
  return __nesc_result;
#line 147
}
#line 147
# 65 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void )
#line 65
{
  uint8_t i;

#line 67
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 1;
  for (i = 0; i < 1; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == (void *)0 || 
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current / 8] & (1 << /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current % 8)) 
        {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 1;
        }
      else {
          break;
        }
    }
  if (i >= 1) {
#line 78
    /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
    }
}

#line 174
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void )
#line 174
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket();
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current < 1) {
      error_t nextErr;
      message_t *nextMsg = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg;
      am_id_t nextId = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(nextMsg);
      am_addr_t nextDest = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(nextMsg);
      uint8_t len = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(nextMsg);

#line 182
      nextErr = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(nextId, nextDest, nextMsg, len);
      if (nextErr != SUCCESS) {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask();
        }
    }
}

# 59 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__SubPacket__maxPayloadLength(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = CC2420XDriverLayerP__RadioPacket__maxPayloadLength();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 105 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MetadataFlagsLayerC.nc"
static inline uint8_t /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__maxPayloadLength(void )
{
  return /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__SubPacket__maxPayloadLength();
}

# 59 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__SubPacket__maxPayloadLength(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__maxPayloadLength();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 131 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TimeStampingLayerP.nc"
static inline uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__maxPayloadLength(void )
{
  return /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__SubPacket__maxPayloadLength();
}

# 59 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__maxPayloadLength(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__RadioPacket__maxPayloadLength();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 297 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline uint8_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__RadioPacket__maxPayloadLength(void )
{
  return /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__maxPayloadLength() - sizeof(ieee154_simple_header_t );
}

# 59 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubPacket__maxPayloadLength(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__RadioPacket__maxPayloadLength();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 169 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TinyosNetworkLayerC.nc"
static inline uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosPacket__maxPayloadLength(void )
{
  return /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubPacket__maxPayloadLength() - /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__PAYLOAD_OFFSET;
}

# 59 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubPacket__maxPayloadLength(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosPacket__maxPayloadLength();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 240 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
static inline uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__RadioPacket__maxPayloadLength(void )
{
  return /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubPacket__maxPayloadLength() - sizeof(activemessage_header_t );
}

#line 272
static inline uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Packet__maxPayloadLength(void )
{
  return /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__RadioPacket__maxPayloadLength();
}

# 97 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__createDataFrame(message_t *msg)
{

  __nesc_hton_leuint16(/*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->fcf.nxdata, (__nesc_ntoh_leuint16(/*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->fcf.nxdata) & /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_DATA_FRAME_PRESERVE)
   | /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_DATA_FRAME_VALUE);
}

# 69 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static void CC2420XRadioP__Ieee154PacketLayer__createDataFrame(message_t *msg){
#line 69
  /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__createDataFrame(msg);
#line 69
}
#line 69
# 192 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline error_t CC2420XRadioP__ActiveMessageConfig__checkFrame(message_t *msg)
{
  if (!CC2420XRadioP__Ieee154PacketLayer__isDataFrame(msg)) {
    CC2420XRadioP__Ieee154PacketLayer__createDataFrame(msg);
    }
  return SUCCESS;
}

# 63 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageConfig.nc"
inline static error_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Config__checkFrame(message_t *msg){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420XRadioP__ActiveMessageConfig__checkFrame(msg);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 193 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setSrcAddr(message_t *msg, uint16_t addr)
{
  __nesc_hton_leuint16(/*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->src.nxdata, addr);
}

# 156 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static void CC2420XRadioP__Ieee154PacketLayer__setSrcAddr(message_t *msg, uint16_t addr){
#line 156
  /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setSrcAddr(msg, addr);
#line 156
}
#line 156
# 177 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline void CC2420XRadioP__ActiveMessageConfig__setSource(message_t *msg, am_addr_t addr)
{
  CC2420XRadioP__Ieee154PacketLayer__setSrcAddr(msg, addr);
}

# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageConfig.nc"
inline static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Config__setSource(message_t *msg, am_addr_t addr){
#line 49
  CC2420XRadioP__ActiveMessageConfig__setSource(msg, addr);
#line 49
}
#line 49
# 194 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
static __inline void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__setSource(message_t *msg, am_addr_t addr)
{
  /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Config__setSource(msg, addr);
}

# 173 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDestPan(message_t *msg, uint16_t pan)
{
  __nesc_hton_leuint16(/*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->destpan.nxdata, pan);
}

# 136 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static void CC2420XRadioP__Ieee154PacketLayer__setDestPan(message_t *msg, uint16_t pan){
#line 136
  /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDestPan(msg, pan);
#line 136
}
#line 136
# 187 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline void CC2420XRadioP__ActiveMessageConfig__setGroup(message_t *msg, am_group_t grp)
{
  CC2420XRadioP__Ieee154PacketLayer__setDestPan(msg, grp);
}

# 55 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageConfig.nc"
inline static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Config__setGroup(message_t *msg, am_group_t grp){
#line 55
  CC2420XRadioP__ActiveMessageConfig__setGroup(msg, grp);
#line 55
}
#line 55
# 214 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
static __inline void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__setGroup(message_t *msg, am_group_t grp)
{
  /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Config__setGroup(msg, grp);
}

# 55 "/opt/tinyos-2.1.2/tos/interfaces/ActiveMessageAddress.nc"
inline static am_group_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__ActiveMessageAddress__amGroup(void ){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amGroup();
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 168 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
static __inline am_group_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__localGroup(void )
{
  return /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__ActiveMessageAddress__amGroup();
}

#line 122
static inline void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SendNotifier__default__aboutToSend(am_id_t id, am_addr_t addr, message_t *msg)
{
}

# 59 "/opt/tinyos-2.1.2/tos/interfaces/SendNotifier.nc"
inline static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SendNotifier__aboutToSend(am_id_t arg_0x2b395a917df8, am_addr_t dest, message_t * msg){
#line 59
    /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SendNotifier__default__aboutToSend(arg_0x2b395a917df8, dest, msg);
#line 59
}
#line 59
# 64 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline bool /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 64
{
  /* atomic removed: atomic calls only */
#line 65
  {
    unsigned char __nesc_temp = 
#line 65
    /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__resQ[id] != /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY || /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qTail == id;

#line 65
    return __nesc_temp;
  }
}

#line 82
static inline error_t /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id)
#line 82
{
  /* atomic removed: atomic calls only */
#line 83
  {
    if (!/*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(id)) {
        if (/*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qHead == /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
          /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qHead = id;
          }
        else {
#line 88
          /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__resQ[/*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qTail] = id;
          }
#line 89
        /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qTail = id;
        {
          unsigned char __nesc_temp = 
#line 90
          SUCCESS;

#line 90
          return __nesc_temp;
        }
      }
#line 92
    {
      unsigned char __nesc_temp = 
#line 92
      EBUSY;

#line 92
      return __nesc_temp;
    }
  }
}

# 79 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
inline static error_t /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__enqueue(resource_client_id_t id){
#line 79
  unsigned char __nesc_result;
#line 79

#line 79
  __nesc_result = /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(id);
#line 79

#line 79
  return __nesc_result;
#line 79
}
#line 79
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 167 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(uint8_t id)
#line 167
{
}

# 53 "/opt/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
inline static void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__requested(uint8_t arg_0x2b395a9fe1a0){
#line 53
    /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(arg_0x2b395a9fe1a0);
#line 53
}
#line 53
# 71 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline error_t /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__request(uint8_t id)
#line 71
{
  /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__requested(/*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 73
    {
      if (/*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__state == /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_IDLE) {
          /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__state = /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_GRANTING;
          /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__reqResId = id;
          /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__postTask();
          {
            unsigned char __nesc_temp = 
#line 78
            SUCCESS;

            {
#line 78
              __nesc_atomic_end(__nesc_atomic); 
#line 78
              return __nesc_temp;
            }
          }
        }
#line 80
      {
        unsigned char __nesc_temp = 
#line 80
        /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__enqueue(id);

        {
#line 80
          __nesc_atomic_end(__nesc_atomic); 
#line 80
          return __nesc_temp;
        }
      }
    }
#line 83
    __nesc_atomic_end(__nesc_atomic); }
}

# 88 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__request(0U);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
#line 120
inline static error_t /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__release(0U);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
inline static error_t /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__SubSend__send(message_t *msg){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosSend__send(msg);
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 171 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 171
{
}

# 59 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
inline static void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x2b395a9fd6e0){
#line 59
    /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(arg_0x2b395a9fd6e0);
#line 59
}
#line 59
# 169 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id)
#line 169
{
}

# 61 "/opt/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
inline static void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__immediateRequested(uint8_t arg_0x2b395a9fe1a0){
#line 61
    /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__immediateRequested(arg_0x2b395a9fe1a0);
#line 61
}
#line 61
# 84 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline error_t /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__immediateRequest(uint8_t id)
#line 84
{
  /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__immediateRequested(/*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 86
    {
      if (/*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__state == /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_IDLE) {
          /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__state = /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_BUSY;
          /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId = id;
          /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(/*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId);
          {
            unsigned char __nesc_temp = 
#line 91
            SUCCESS;

            {
#line 91
              __nesc_atomic_end(__nesc_atomic); 
#line 91
              return __nesc_temp;
            }
          }
        }
#line 93
      {
        unsigned char __nesc_temp = 
#line 93
        FAIL;

        {
#line 93
          __nesc_atomic_end(__nesc_atomic); 
#line 93
          return __nesc_temp;
        }
      }
    }
#line 96
    __nesc_atomic_end(__nesc_atomic); }
}

# 97 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__immediateRequest(0U);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 53 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/AutoResourceAcquireLayerC.nc"
static inline error_t /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__BareSend__send(message_t *msg)
{
  if (/*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__immediateRequest() == SUCCESS) 
    {
      error_t result = /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__SubSend__send(msg);

#line 58
      if (result != SUCCESS) {
        /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__release();
        }
      return result;
    }

  /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__pending = msg;
  return /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__request();
}

# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
inline static error_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubSend__send(message_t *msg){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__BareSend__send(msg);
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 79 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
inline static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Tasklet__resume(void ){
#line 79
  TaskletC__Tasklet__resume();
#line 79
}
#line 79
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__sendTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__sendTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 94 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/TaskletC.nc"
static __inline void TaskletC__Tasklet__suspend(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 96
    ++TaskletC__state;
#line 96
    __nesc_atomic_end(__nesc_atomic); }
}

# 72 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
inline static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Tasklet__suspend(void ){
#line 72
  TaskletC__Tasklet__suspend();
#line 72
}
#line 72
# 230 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MessageBufferLayerP.nc"
static inline error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Send__send(message_t *msg)
{
  error_t result;

  /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Tasklet__suspend();

  if (/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__state != /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__STATE_READY) {
    result = EBUSY;
    }
  else {
      /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__txMsg = msg;
      /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__state = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__STATE_TX_PENDING;
      /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__retries = 0;
      /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__sendTask__postTask();
      result = SUCCESS;
    }

  /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Tasklet__resume();

  return result;
}

# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
inline static error_t /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__SubSend__send(message_t *msg){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Send__send(msg);
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 163 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDSN(message_t *msg, uint8_t dsn)
{
  __nesc_hton_leuint8(/*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->dsn.nxdata, dsn);
}

# 125 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static void CC2420XRadioP__Ieee154PacketLayer__setDSN(message_t *msg, uint8_t dsn){
#line 125
  /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDSN(msg, dsn);
#line 125
}
#line 125
# 143 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline void CC2420XRadioP__UniqueConfig__setSequenceNumber(message_t *msg, uint8_t dsn)
{
  CC2420XRadioP__Ieee154PacketLayer__setDSN(msg, dsn);
}

# 52 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/UniqueConfig.nc"
inline static void /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__UniqueConfig__setSequenceNumber(message_t *msg, uint8_t number){
#line 52
  CC2420XRadioP__UniqueConfig__setSequenceNumber(msg, number);
#line 52
}
#line 52
# 69 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/UniqueLayerP.nc"
static inline error_t /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__Send__send(message_t *msg)
{
  /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__UniqueConfig__setSequenceNumber(msg, ++/*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__sequenceNumber);
  return /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__SubSend__send(msg);
}

# 46 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
inline static error_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubSend__send(message_t *msg){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__Send__send(msg);
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 60 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline bool /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void )
#line 60
{
  /* atomic removed: atomic calls only */
#line 61
  {
    unsigned char __nesc_temp = 
#line 61
    /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qHead == /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

#line 61
    return __nesc_temp;
  }
}

# 53 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
inline static bool /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__isEmpty(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 68 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void )
#line 68
{
  /* atomic removed: atomic calls only */
#line 69
  {
    if (/*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qHead != /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
        uint8_t id = /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qHead;

#line 72
        /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qHead = /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__resQ[/*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qHead];
        if (/*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qHead == /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
          /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__qTail = /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
          }
#line 75
        /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__resQ[id] = /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 76
          id;

#line 76
          return __nesc_temp;
        }
      }
#line 78
    {
      unsigned char __nesc_temp = 
#line 78
      /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

#line 78
      return __nesc_temp;
    }
  }
}

# 70 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__dequeue(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 173 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id)
#line 173
{
}

# 65 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
inline static void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0x2b395a9fd6e0){
#line 65
    /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(arg_0x2b395a9fd6e0);
#line 65
}
#line 65
# 175 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(uint8_t id)
#line 175
{
#line 175
  return FAIL;
}

# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(uint8_t arg_0x2b395af86898){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  switch (arg_0x2b395af86898) {
#line 120
    case /*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 120
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(/*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 120
      break;
#line 120
    default:
#line 120
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(arg_0x2b395af86898);
#line 120
      break;
#line 120
    }
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 116 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(uint8_t id)
#line 116
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(id);
}

# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420XDriverLayerP__SpiResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(/*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 85 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput();
#line 85
}
#line 85
# 54 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*HplCC2420XC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput();
}

# 46 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__CSN__makeOutput(void ){
#line 46
  /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput();
#line 46
}
#line 46
# 408 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static inline void CC2420XDriverLayerP__SpiResource__granted(void )
{

  CC2420XDriverLayerP__CSN__makeOutput();
  CC2420XDriverLayerP__CSN__set();

  if (CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_VR_ON) 
    {
      CC2420XDriverLayerP__initRadio();
      CC2420XDriverLayerP__SpiResource__release();
    }
  else {
    CC2420XDriverLayerP__Tasklet__schedule();
    }
}

# 180 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(uint8_t id)
#line 180
{
}

# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(uint8_t arg_0x2b395af389f8){
#line 102
  switch (arg_0x2b395af389f8) {
#line 102
    case /*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 102
      CC2420XDriverLayerP__SpiResource__granted();
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(arg_0x2b395af389f8);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 130 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(uint8_t id)
#line 130
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(id);
}

# 202 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id)
#line 202
{
}

# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(uint8_t arg_0x2b395b0dd2f0){
#line 102
  switch (arg_0x2b395b0dd2f0) {
#line 102
    case /*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 102
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(/*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(arg_0x2b395b0dd2f0);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 190 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void )
#line 190
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 191
    {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
    }
#line 194
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
}

# 60 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void )
#line 60
{
  /* atomic removed: atomic calls only */
#line 61
  {
    unsigned char __nesc_temp = 
#line 61
    /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

#line 61
    return __nesc_temp;
  }
}

# 53 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 68 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void )
#line 68
{
  /* atomic removed: atomic calls only */
#line 69
  {
    if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
        uint8_t id = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead;

#line 72
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead];
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
          }
#line 75
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[id] = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 76
          id;

#line 76
          return __nesc_temp;
        }
      }
#line 78
    {
      unsigned char __nesc_temp = 
#line 78
      /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

#line 78
      return __nesc_temp;
    }
  }
}

# 70 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 97 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(bool reset){
#line 97
  HplMsp430Usart0P__Usart__resetUsart(reset);
#line 97
}
#line 97
#line 158
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi(void ){
#line 158
  HplMsp430Usart0P__Usart__disableSpi();
#line 158
}
#line 158
# 124 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 124
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(TRUE);
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi();
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(FALSE);
}

# 218 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id)
#line 218
{
}

# 65 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0x2b395b0da6e0){
#line 65
  switch (arg_0x2b395b0da6e0) {
#line 65
    case /*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 65
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(/*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 65
      break;
#line 65
    default:
#line 65
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(arg_0x2b395b0da6e0);
#line 65
      break;
#line 65
    }
#line 65
}
#line 65
# 67 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 1);
}

# 99 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 2);
}

# 99 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 3);
}

# 99 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc();
#line 99
}
#line 99
# 208 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void )
#line 208
{
}

# 46 "/opt/tinyos-2.1.2/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted();
#line 46
}
#line 46
# 251 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error)
#line 251
{
}

# 82 "/opt/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(uint8_t arg_0x2b395af87ae8, uint8_t * txBuf, uint8_t * rxBuf, uint16_t len, error_t error){
#line 82
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(arg_0x2b395af87ae8, txBuf, rxBuf, len, error);
#line 82
}
#line 82
# 244 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void )
#line 244
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_client, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len, 
  SUCCESS);
}

#line 227
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void )
#line 227
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 228
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone();
#line 228
    __nesc_atomic_end(__nesc_atomic); }
}

# 1102 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static inline void CC2420XDriverLayerP__releaseSpi__runTask(void )
{
  CC2420XDriverLayerP__SpiResource__release();
}

# 89 "/opt/tinyos-2.1.2/tos/system/RandomMlcgC.nc"
static inline uint16_t RandomMlcgC__Random__rand16(void )
#line 89
{
  return (uint16_t )RandomMlcgC__Random__rand32();
}

# 52 "/opt/tinyos-2.1.2/tos/interfaces/Random.nc"
inline static uint16_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__Random__rand16(void ){
#line 52
  unsigned int __nesc_result;
#line 52

#line 52
  __nesc_result = RandomMlcgC__Random__rand16();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 78 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/RandomCollisionLayerP.nc"
static inline void /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__calcNextRandom__runTask(void )
{
  uint16_t a = /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__Random__rand16();

#line 81
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 81
    /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__nextRandom = a;
#line 81
    __nesc_atomic_end(__nesc_atomic); }
}

# 176 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MessageBufferLayerP.nc"
static inline void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioChannel__default__setChannelDone(void )
{
}

# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioChannel.nc"
inline static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioChannel__setChannelDone(void ){
#line 48
  /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioChannel__default__setChannelDone();
#line 48
}
#line 48
# 76 "BlinkToRadioC.nc"
static inline void BlinkToRadioC__AMControl__stopDone(error_t err)
#line 76
{
}

# 138 "/opt/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
inline static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__SplitControl__stopDone(error_t error){
#line 138
  BlinkToRadioC__AMControl__stopDone(error);
#line 138
}
#line 138
#line 104
inline static error_t BlinkToRadioC__AMControl__start(void ){
#line 104
  unsigned char __nesc_result;
#line 104

#line 104
  __nesc_result = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__SplitControl__start();
#line 104

#line 104
  return __nesc_result;
#line 104
}
#line 104
# 68 "BlinkToRadioC.nc"
static inline void BlinkToRadioC__AMControl__startDone(error_t err)
#line 68
{
  if (err == SUCCESS) {
    }
  else {
      BlinkToRadioC__AMControl__start();
    }
}

# 113 "/opt/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
inline static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__SplitControl__startDone(error_t error){
#line 113
  BlinkToRadioC__AMControl__startDone(error);
#line 113
}
#line 113
# 144 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MessageBufferLayerP.nc"
static inline void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__stateDoneTask__runTask(void )
{
  uint8_t s;

  s = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__state;


  /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__state = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__STATE_READY;

  if (s == /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__STATE_TURN_ON) {
    /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__SplitControl__startDone(SUCCESS);
    }
  else {
#line 155
    if (s == /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__STATE_TURN_OFF) {
      /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__SplitControl__stopDone(SUCCESS);
      }
    else {
#line 157
      if (s == /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__STATE_CHANNEL) {
        /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioChannel__setChannelDone();
        }
      else {
#line 160
        /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__state = s;
        }
      }
    }
}

# 585 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static inline error_t CC2420XDriverLayerP__RadioState__turnOn(void )
{
  if (CC2420XDriverLayerP__cmd != CC2420XDriverLayerP__CMD_NONE || (CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_PD && !CC2420XDriverLayerP__RadioAlarm__isFree())) {
    return EBUSY;
    }
  else {
#line 589
    if (CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_RX_ON) {
      return EALREADY;
      }
    }








  CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_TURNON;
  CC2420XDriverLayerP__Tasklet__schedule();

  return SUCCESS;
}

# 56 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioState.nc"
inline static error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioState__turnOn(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = CC2420XDriverLayerP__RadioState__turnOn();
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 132 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154MessageLayerC.nc"
static inline void /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Ieee154Send__default__sendDone(message_t *msg, error_t error)
{
}

# 97 "/opt/tinyos-2.1.2/tos/interfaces/Ieee154Send.nc"
inline static void /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Ieee154Send__sendDone(message_t *msg, error_t error){
#line 97
  /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Ieee154Send__default__sendDone(msg, error);
#line 97
}
#line 97
# 127 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154MessageLayerC.nc"
static inline void /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__SubSend__sendDone(message_t *msg, error_t error)
{
  /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Ieee154Send__sendDone(msg, error);
}

# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
inline static void /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__Ieee154Send__sendDone(message_t *msg, error_t error){
#line 54
  /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__SubSend__sendDone(msg, error);
#line 54
}
#line 54
# 189 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err)
#line 189
{





  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 1) {
      return;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == msg) {
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current, msg, err);
    }
  else {
      ;
    }
}

# 110 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
inline static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMSend__sendDone(am_id_t arg_0x2b395a91a5f0, message_t * msg, error_t error){
#line 110
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(arg_0x2b395a91a5f0, msg, error);
#line 110
}
#line 110
# 98 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
static __inline void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubSend__sendDone(message_t *msg, error_t error)
{
  /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMSend__sendDone(/*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__type(msg), msg, error);
}

# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
inline static void /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__BareSend__sendDone(message_t *msg, error_t error){
#line 54
  /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubSend__sendDone(msg, error);
#line 54
}
#line 54
# 78 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/AutoResourceAcquireLayerC.nc"
static inline void /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__SubSend__sendDone(message_t *msg, error_t result)
{
  /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__release();
  /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__BareSend__sendDone(msg, result);
}

# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
inline static void /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosSend__sendDone(message_t *msg, error_t error){
#line 54
  /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__SubSend__sendDone(msg, error);
#line 54
}
#line 54
# 127 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TinyosNetworkLayerC.nc"
static inline network_header_t */*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__getHeader(message_t *msg)
{
  return (void *)msg + /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubPacket__headerLength(msg);
}

#line 214
static inline void /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubSend__sendDone(message_t *msg, error_t result)
{
  if (__nesc_ntoh_leuint8(/*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__getHeader(msg)->network.nxdata) == 0x3f) {
    /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosSend__sendDone(msg, result);
    }
  else {
#line 219
    /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__Ieee154Send__sendDone(msg, result);
    }
}

# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
inline static void /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__Send__sendDone(message_t *msg, error_t error){
#line 54
  /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubSend__sendDone(msg, error);
#line 54
}
#line 54
# 80 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/UniqueLayerP.nc"
static inline void /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__SubSend__sendDone(message_t *msg, error_t error)
{
  /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__Send__sendDone(msg, error);
}

# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareSend.nc"
inline static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Send__sendDone(message_t *msg, error_t error){
#line 54
  /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__SubSend__sendDone(msg, error);
#line 54
}
#line 54
# 234 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline uint16_t CC2420XRadioP__RandomCollisionConfig__getInitialBackoff(message_t *msg)
{
  return (uint16_t )(9920 * 1);
}

# 40 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/RandomCollisionConfig.nc"
inline static uint16_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__Config__getInitialBackoff(message_t *msg){
#line 40
  unsigned int __nesc_result;
#line 40

#line 40
  __nesc_result = CC2420XRadioP__RandomCollisionConfig__getInitialBackoff(msg);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 98 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/RandomCollisionLayerP.nc"
static inline error_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioSend__send(message_t *msg)
{
  if (/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__state != /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__STATE_READY || !/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__isFree()) {
    return EBUSY;
    }
  /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__txMsg = msg;
  /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__state = /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__STATE_TX_PENDING_FIRST;
  /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__wait(/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__getBackoff(/*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__Config__getInitialBackoff(msg)));

  return SUCCESS;
}

# 48 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
inline static error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioSend__send(message_t *msg){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioSend__send(msg);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 189 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MessageBufferLayerP.nc"
static inline void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__sendTask__runTask(void )
{
  bool done = FALSE;

  /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Tasklet__suspend();

  for (; 0; ) ;

  if (/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__state == /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__STATE_TX_PENDING && ++/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__retries <= /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__MAX_RETRIES) 
    {
      /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__txError = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioSend__send(/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__txMsg);
      if (/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__txError == SUCCESS) {
        /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__state = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__STATE_TX_SEND;
        }
      else {
#line 203
        /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__state = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__STATE_TX_RETRY;
        }
    }
  else {
      /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__state = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__STATE_READY;
      done = TRUE;
    }

  /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Tasklet__resume();

  if (done) {
    /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Send__sendDone(/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__txMsg, /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__txError);
    }
}

# 96 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TinyosNetworkLayerC.nc"
static inline uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__Ieee154Packet__payloadLength(message_t *msg)
{
  return /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubPacket__payloadLength(msg);
}

# 49 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__RadioPacket__payloadLength(message_t *msg){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__Ieee154Packet__payloadLength(msg);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 68 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154MessageLayerC.nc"
static inline uint8_t /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Packet__payloadLength(message_t *msg)
{
  return /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__RadioPacket__payloadLength(msg);
}

# 91 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TinyosNetworkLayerC.nc"
static inline uint8_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__Ieee154Packet__headerLength(message_t *msg)
{
  return /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubPacket__headerLength(msg);
}

# 43 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__RadioPacket__headerLength(message_t *msg){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__Ieee154Packet__headerLength(msg);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 56 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154MessageLayerC.nc"
static inline void */*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__getPayload(message_t *msg)
{
  return (void *)msg + /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__RadioPacket__headerLength(msg);
}

#line 151
static inline message_t */*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Ieee154Receive__default__receive(message_t *msg, void *payload, uint8_t len)
{
  return msg;
}

# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
inline static message_t * /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Ieee154Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Ieee154Receive__default__receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 55 "/opt/tinyos-2.1.2/tos/interfaces/ActiveMessageAddress.nc"
inline static am_group_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__ActiveMessageAddress__amGroup(void ){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amGroup();
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 217 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline ieee154_panid_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__localPan(void )
{
  return /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__ActiveMessageAddress__amGroup();
}

#line 168
static inline uint16_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDestPan(message_t *msg)
{
  return __nesc_ntoh_leuint16(/*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->destpan.nxdata);
}

# 72 "/opt/tinyos-2.1.2/tos/system/ActiveMessageAddressC.nc"
static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void )
#line 72
{
  return ActiveMessageAddressC__amAddress();
}

# 50 "/opt/tinyos-2.1.2/tos/interfaces/ActiveMessageAddress.nc"
inline static am_addr_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__ActiveMessageAddress__amAddress(void ){
#line 50
  unsigned int __nesc_result;
#line 50

#line 50
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amAddress();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 212 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline ieee154_saddr_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__localAddr(void )
{
  return /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__ActiveMessageAddress__amAddress();
}






static inline bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isForMe(message_t *msg)
{
  ieee154_saddr_t addr = /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDestAddr(msg);

#line 225
  return (addr == /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__localAddr() || addr == IEEE154_BROADCAST_ADDR)
   && /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDestPan(msg) == /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__localPan();
}

# 185 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static bool /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Ieee154PacketLayer__isForMe(message_t *msg){
#line 185
  unsigned char __nesc_result;
#line 185

#line 185
  __nesc_result = /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isForMe(msg);
#line 185

#line 185
  return __nesc_result;
#line 185
}
#line 185
# 142 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154MessageLayerC.nc"
static inline message_t */*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__SubReceive__receive(message_t *msg)
{
  if (/*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Ieee154PacketLayer__isForMe(msg)) {
    return /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Ieee154Receive__receive(msg, 
    /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__getPayload(msg), /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__Packet__payloadLength(msg));
    }
  else {
#line 148
    return msg;
    }
}

# 42 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareReceive.nc"
inline static message_t */*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__Ieee154Receive__receive(message_t *msg){
#line 42
  nx_struct message_t *__nesc_result;
#line 42

#line 42
  __nesc_result = /*CC2420XRadioC.Ieee154MessageLayerC*/Ieee154MessageLayerC__0__SubReceive__receive(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 156 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
static inline message_t */*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SnoopDefault__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
{
  return msg;
}

# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
inline static message_t * /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SnoopDefault__receive(am_id_t arg_0x2b395a9156b8, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
    __nesc_result = /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SnoopDefault__default__receive(arg_0x2b395a9156b8, msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 151 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
static inline message_t */*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Snoop__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
{
  return /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SnoopDefault__receive(id, msg, payload, len);
#line 153
  ;
}

# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
inline static message_t * /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Snoop__receive(am_id_t arg_0x2b395a9172f0, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
    __nesc_result = /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Snoop__default__receive(arg_0x2b395a9172f0, msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 146 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
static inline message_t */*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__ReceiveDefault__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
{
  return msg;
}

# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
inline static message_t * /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__ReceiveDefault__receive(am_id_t arg_0x2b395a916a38, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
    __nesc_result = /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__ReceiveDefault__default__receive(arg_0x2b395a916a38, msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 286 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_uint8(void * target, uint8_t value)
#line 286
{
  uint8_t *base = target;

#line 288
  base[0] = value;
  return value;
}

# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 86 "/opt/tinyos-2.1.2/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
}

# 109 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void ){
#line 109
  unsigned long __nesc_result;
#line 109

#line 109
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 96 "/opt/tinyos-2.1.2/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void )
{
#line 97
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow();
}

# 136 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void ){
#line 136
  unsigned long __nesc_result;
#line 136

#line 136
  __nesc_result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow();
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 144 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

#line 147
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}






static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, TRUE);
}

# 73 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static void BlinkToRadioC__Timer0__startOneShot(uint32_t dt){
#line 73
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(0U, dt);
#line 73
}
#line 73
# 63 "/opt/tinyos-2.1.2/tos/interfaces/PacketTimeStamp.nc"
inline static BlinkToRadioC__PacketTimeStamp__size_type BlinkToRadioC__PacketTimeStamp__timestamp(message_t * msg){
#line 63
  unsigned long __nesc_result;
#line 63

#line 63
  __nesc_result = /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__PacketTimeStampRadio__timestamp(msg);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle(void )
#line 58
{
#line 58
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 58
    * (volatile uint8_t * )49U ^= 0x01 << 4;
#line 58
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__toggle(void ){
#line 58
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle();
#line 58
}
#line 58
# 50 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle(void )
#line 50
{
#line 50
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__toggle();
}

# 42 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__toggle(void ){
#line 42
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle();
#line 42
}
#line 42
# 84 "/opt/tinyos-2.1.2/tos/system/LedsP.nc"
static inline void LedsP__Leds__led0Toggle(void )
#line 84
{
  LedsP__Led0__toggle();
  ;
#line 86
  ;
}

# 67 "/opt/tinyos-2.1.2/tos/interfaces/Leds.nc"
inline static void BlinkToRadioC__Leds__led0Toggle(void ){
#line 67
  LedsP__Leds__led0Toggle();
#line 67
}
#line 67
# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle(void )
#line 58
{
#line 58
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 58
    * (volatile uint8_t * )49U ^= 0x01 << 5;
#line 58
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__toggle(void ){
#line 58
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle();
#line 58
}
#line 58
# 50 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle(void )
#line 50
{
#line 50
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__toggle();
}

# 42 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__toggle(void ){
#line 42
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle();
#line 42
}
#line 42
# 99 "/opt/tinyos-2.1.2/tos/system/LedsP.nc"
static inline void LedsP__Leds__led1Toggle(void )
#line 99
{
  LedsP__Led1__toggle();
  ;
#line 101
  ;
}

# 83 "/opt/tinyos-2.1.2/tos/interfaces/Leds.nc"
inline static void BlinkToRadioC__Leds__led1Toggle(void ){
#line 83
  LedsP__Leds__led1Toggle();
#line 83
}
#line 83
# 94 "BlinkToRadioC.nc"
static inline message_t *BlinkToRadioC__Receive__receive(message_t *msg, void *payload, uint8_t len)
#line 94
{
  if (len == sizeof(BlinkToRadioMsg )) {
      BlinkToRadioMsg *btrpkt = (BlinkToRadioMsg *)payload;

#line 97
      BlinkToRadioC__Leds__led1Toggle();


      if (!BlinkToRadioC__busy) {

          if (__nesc_ntoh_uint8(btrpkt->nodeid.nxdata) == 10 || __nesc_ntoh_uint8(btrpkt->nodeid.nxdata) == 35) {

              BlinkToRadioC__received_counter = __nesc_ntoh_uint8(btrpkt->counter.nxdata);
              __nesc_hton_uint8(btrpkt->nodeid.nxdata, TOS_NODE_ID);

              BlinkToRadioC__Leds__led0Toggle();

              if (1 && BlinkToRadioC__received_counter <= 50) {

                  __nesc_hton_uint32(btrpkt->receiveTS.nxdata, BlinkToRadioC__receiveTS);
                  BlinkToRadioC__receiveTS = BlinkToRadioC__PacketTimeStamp__timestamp(msg);
                  __nesc_hton_uint8(btrpkt->nodeid.nxdata, TOS_NODE_ID);

                  __nesc_hton_uint32(btrpkt->sendTS_of_member.nxdata, BlinkToRadioC__resendTS);

                  BlinkToRadioC__pktptr = msg;


                  BlinkToRadioC__Timer0__startOneShot(1);
                }
            }
        }
    }


  return msg;
}

# 141 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
static inline message_t */*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Receive__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
{
  return /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__ReceiveDefault__receive(id, msg, payload, len);
#line 143
  ;
}

# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
inline static message_t * /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Receive__receive(am_id_t arg_0x2b395a919780, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  switch (arg_0x2b395a919780) {
#line 78
    case 6:
#line 78
      __nesc_result = BlinkToRadioC__Receive__receive(msg, payload, len);
#line 78
      break;
#line 78
    default:
#line 78
      __nesc_result = /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Receive__default__receive(arg_0x2b395a919780, msg, payload, len);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 50 "/opt/tinyos-2.1.2/tos/interfaces/ActiveMessageAddress.nc"
inline static am_addr_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__ActiveMessageAddress__amAddress(void ){
#line 50
  unsigned int __nesc_result;
#line 50

#line 50
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amAddress();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 163 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
static __inline am_addr_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__address(void )
{
  return /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__ActiveMessageAddress__amAddress();
}






static __inline bool /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__isForMe(message_t *msg)
{
  am_addr_t addr = /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__destination(msg);

#line 176
  return addr == /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__address() || addr == AM_BROADCAST_ADDR;
}

#line 225
static inline uint8_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__RadioPacket__headerLength(message_t *msg)
{
  return /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubPacket__headerLength(msg) + sizeof(activemessage_header_t );
}

#line 72
static inline void */*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__getPayload(message_t *msg)
{
  return (void *)msg + /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__RadioPacket__headerLength(msg);
}

#line 128
static inline message_t */*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubReceive__receive(message_t *msg)
{
  am_id_t id = /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__type(msg);
  void *payload = /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__getPayload(msg);
  uint8_t len = /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Packet__payloadLength(msg);

  msg = /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__isForMe(msg) ? 
  /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Receive__receive(id, msg, payload, len) : 
  /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Snoop__receive(id, msg, payload, len);

  return msg;
}

# 42 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareReceive.nc"
inline static message_t */*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosReceive__receive(message_t *msg){
#line 42
  nx_struct message_t *__nesc_result;
#line 42

#line 42
  __nesc_result = /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubReceive__receive(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 222 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TinyosNetworkLayerC.nc"
static inline message_t */*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubReceive__receive(message_t *msg)
{
  if (__nesc_ntoh_leuint8(/*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__getHeader(msg)->network.nxdata) == 0x3f) {
    return /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosReceive__receive(msg);
    }
  else {
#line 227
    return /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__Ieee154Receive__receive(msg);
    }
}

# 42 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/BareReceive.nc"
inline static message_t */*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Receive__receive(message_t *msg){
#line 42
  nx_struct message_t *__nesc_result;
#line 42

#line 42
  __nesc_result = /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubReceive__receive(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 322 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MessageBufferLayerP.nc"
static inline void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__deliverTask__runTask(void )
{

  for (; ; ) 
    {
      message_t *msg;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          if (/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueueSize == 0) {
            {
#line 332
              __nesc_atomic_end(__nesc_atomic); 
#line 332
              return;
            }
            }
#line 334
          msg = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueue[/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueueHead];
        }
#line 335
        __nesc_atomic_end(__nesc_atomic); }

      msg = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Receive__receive(msg);

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueue[/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueueHead] = msg;

          if (++/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueueHead >= /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RECEIVE_QUEUE_SIZE) {
            /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueueHead = 0;
            }
          --/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueueSize;
        }
#line 347
        __nesc_atomic_end(__nesc_atomic); }
    }
}

# 68 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/AutoResourceAcquireLayerC.nc"
static inline void /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__granted(void )
{
  error_t result = /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__SubSend__send(/*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__pending);

#line 71
  if (result != SUCCESS) 
    {
      /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__release();
      /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__BareSend__sendDone(/*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__pending, result);
    }
}

# 165 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__default__granted(uint8_t id)
#line 165
{
}

# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__granted(uint8_t arg_0x2b395a9be020){
#line 102
  switch (arg_0x2b395a9be020) {
#line 102
    case 0U:
#line 102
      /*CC2420XRadioC.AutoResourceAcquireLayerC*/AutoResourceAcquireLayerC__0__Resource__granted();
#line 102
      break;
#line 102
    default:
#line 102
      /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__default__granted(arg_0x2b395a9be020);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 155 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline void /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__runTask(void )
#line 155
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 156
    {
      /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId = /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__reqResId;
      /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__state = /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_BUSY;
    }
#line 159
    __nesc_atomic_end(__nesc_atomic); }
  /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(/*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId);
  /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__granted(/*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId);
}

# 103 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 58 "/opt/tinyos-2.1.2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(t0, dt);
}

#line 93
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 94
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(t0, dt, TRUE);
}

# 129 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt){
#line 129
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(t0, dt);
#line 129
}
#line 129
# 65 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
}

# 73 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 102 "/opt/tinyos-2.1.2/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop();
}

# 73 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 71 "/opt/tinyos-2.1.2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void )
{
#line 72
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop();
}

# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop();
#line 78
}
#line 78
# 100 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint16_t num;

  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop();

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(now);
        }
      else {
#line 135
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(now, min_remaining);
        }
    }
}

# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle(void )
#line 58
{
#line 58
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 58
    * (volatile uint8_t * )49U ^= 0x01 << 6;
#line 58
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle(void ){
#line 58
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle();
#line 58
}
#line 58
# 50 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void )
#line 50
{
#line 50
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle();
}

# 42 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__toggle(void ){
#line 42
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle();
#line 42
}
#line 42
# 114 "/opt/tinyos-2.1.2/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2Toggle(void )
#line 114
{
  LedsP__Led2__toggle();
  ;
#line 116
  ;
}

# 100 "/opt/tinyos-2.1.2/tos/interfaces/Leds.nc"
inline static void BlinkToRadioC__Leds__led2Toggle(void ){
#line 100
  LedsP__Leds__led2Toggle();
#line 100
}
#line 100
# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static void /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubPacket__setPayloadLength(message_t *msg, uint8_t length){
#line 54
  /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__RadioPacket__setPayloadLength(msg, length);
#line 54
}
#line 54
# 164 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TinyosNetworkLayerC.nc"
static inline void /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosPacket__setPayloadLength(message_t *msg, uint8_t length)
{
  /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubPacket__setPayloadLength(msg, length + /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__PAYLOAD_OFFSET);
}

# 54 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubPacket__setPayloadLength(message_t *msg, uint8_t length){
#line 54
  /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosPacket__setPayloadLength(msg, length);
#line 54
}
#line 54
# 235 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
static inline void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__RadioPacket__setPayloadLength(message_t *msg, uint8_t length)
{
  /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubPacket__setPayloadLength(msg, length + sizeof(activemessage_header_t ));
}

#line 267
static inline void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Packet__setPayloadLength(message_t *msg, uint8_t len)
{
  /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__RadioPacket__setPayloadLength(msg, len);
}

# 94 "/opt/tinyos-2.1.2/tos/interfaces/Packet.nc"
inline static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(message_t * msg, uint8_t len){
#line 94
  /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Packet__setPayloadLength(msg, len);
#line 94
}
#line 94
# 90 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
static inline error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len)
#line 91
{
  if (clientId >= 1) {
      return FAIL;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg != (void *)0) {
      return EBUSY;
    }
  ;

  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = msg;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(msg, len);

  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 1) {
      error_t err;
      am_id_t amId = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(msg);
      am_addr_t dest = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(msg);

      ;
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = clientId;

      err = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(amId, dest, msg, len);
      if (err != SUCCESS) {
          ;
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = (void *)0;
        }

      return err;
    }
  else {
      ;
    }
  return SUCCESS;
}

# 75 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static error_t /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(0U, msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 204 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
static __inline void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__setType(message_t *msg, am_id_t type)
{
  __nesc_hton_uint8(/*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__getHeader(msg)->type.nxdata, type);
}

# 162 "/opt/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
inline static void /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(message_t * amsg, am_id_t t){
#line 162
  /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__setType(amsg, t);
#line 162
}
#line 162
# 183 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDestAddr(message_t *msg, uint16_t addr)
{
  __nesc_hton_leuint16(/*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->dest.nxdata, addr);
}

# 146 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static void CC2420XRadioP__Ieee154PacketLayer__setDestAddr(message_t *msg, uint16_t addr){
#line 146
  /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDestAddr(msg, addr);
#line 146
}
#line 146
# 167 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadioP.nc"
static inline void CC2420XRadioP__ActiveMessageConfig__setDestination(message_t *msg, am_addr_t addr)
{
  CC2420XRadioP__Ieee154PacketLayer__setDestAddr(msg, addr);
}

# 43 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageConfig.nc"
inline static void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Config__setDestination(message_t *msg, am_addr_t addr){
#line 43
  CC2420XRadioP__ActiveMessageConfig__setDestination(msg, addr);
#line 43
}
#line 43
# 184 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
static __inline void /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__setDestination(message_t *msg, am_addr_t addr)
{
  /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Config__setDestination(msg, addr);
}

# 103 "/opt/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
inline static void /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 103
  /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__setDestination(amsg, addr);
#line 103
}
#line 103
# 53 "/opt/tinyos-2.1.2/tos/system/AMQueueEntryP.nc"
static inline error_t /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 55
{
  /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(msg, dest);
  /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(msg, 6);
  return /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(msg, len);
}

# 80 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
inline static error_t BlinkToRadioC__AMSend__send(am_addr_t addr, message_t * msg, uint8_t len){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(addr, msg, len);
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 79 "BlinkToRadioC.nc"
static inline void BlinkToRadioC__Timer0__fired(void )
#line 79
{
  if (BlinkToRadioC__AMSend__send(10, BlinkToRadioC__pktptr, sizeof(BlinkToRadioMsg )) == SUCCESS) {
      BlinkToRadioC__busy = TRUE;
      BlinkToRadioC__Leds__led2Toggle();
    }
}

# 204 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 83 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0x2b395a6cd5d8){
#line 83
  switch (arg_0x2b395a6cd5d8) {
#line 83
    case 0U:
#line 83
      BlinkToRadioC__Timer0__fired();
#line 83
      break;
#line 83
    default:
#line 83
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0x2b395a6cd5d8);
#line 83
      break;
#line 83
    }
#line 83
}
#line 83
# 139 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow());
}

# 83 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void ){
#line 83
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired();
#line 83
}
#line 83
# 91 "/opt/tinyos-2.1.2/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 93
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type __nesc_temp = 
#line 93
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

      {
#line 93
        __nesc_atomic_end(__nesc_atomic); 
#line 93
        return __nesc_temp;
      }
    }
#line 95
    __nesc_atomic_end(__nesc_atomic); }
}

# 116 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void ){
#line 116
  unsigned long __nesc_result;
#line 116

#line 116
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm();
#line 116

#line 116
  return __nesc_result;
#line 116
}
#line 116
# 74 "/opt/tinyos-2.1.2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt, FALSE);
    }
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired();
}

# 69 "/opt/tinyos-2.1.2/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 57 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4356 {
#line 57
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t f;
#line 57
    uint16_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__CC2int(x);
}

#line 105
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )354U = /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__compareControl();
}

# 47 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void ){
#line 47
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControlAsCompare();
#line 47
}
#line 47
# 53 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 55 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__Init__init(void )
#line 55
{
  memset(/*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__resQ, /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__NO_ENTRY, sizeof /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__resQ);
  return SUCCESS;
}

# 63 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/UniqueLayerP.nc"
static inline error_t /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__Init__init(void )
{
  /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__sequenceNumber = TOS_NODE_ID << 4;
  return SUCCESS;
}

# 55 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/NeighborhoodP.nc"
static inline error_t NeighborhoodP__Init__init(void )
{
  uint8_t i;

  for (i = 0; i < 5; ++i) 
    NeighborhoodP__nodes[i] = AM_BROADCAST_ADDR;

  return SUCCESS;
}

# 302 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MessageBufferLayerP.nc"
static inline error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__SoftwareInit__init(void )
{
  uint8_t i;

  for (i = 0; i < /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RECEIVE_QUEUE_SIZE; ++i) 
    /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueue[i] = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__receiveQueueData + i;

  return SUCCESS;
}

# 55 "/opt/tinyos-2.1.2/tos/system/RandomMlcgC.nc"
static inline error_t RandomMlcgC__Init__init(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 56
  RandomMlcgC__seed = (uint32_t )(TOS_NODE_ID + 1);

  return SUCCESS;
}

# 57 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4357 {
#line 57
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t f;
#line 57
    uint16_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(x);
}

#line 105
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl();
}

# 47 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare();
#line 47
}
#line 47
# 53 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Init__init(void )
{
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 173 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(uint8_t id)
#line 173
{
#line 173
  return FAIL;
}

# 88 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(uint8_t arg_0x2b395af86898){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  switch (arg_0x2b395af86898) {
#line 88
    case /*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 88
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(/*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 88
      break;
#line 88
    default:
#line 88
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(arg_0x2b395af86898);
#line 88
      break;
#line 88
    }
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 108 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(uint8_t id)
#line 108
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(id);
}

# 88 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420XDriverLayerP__SpiResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(/*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 56 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )29U |= 0x01 << 5;
}

# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.VRENM*/Msp430GpioC__9__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set();
#line 48
}
#line 48
# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.VRENM*/Msp430GpioC__9__GeneralIO__set(void )
#line 48
{
#line 48
  /*HplCC2420XC.VRENM*/Msp430GpioC__9__HplGeneralIO__set();
}

# 40 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__VREN__set(void ){
#line 40
  /*HplCC2420XC.VRENM*/Msp430GpioC__9__GeneralIO__set();
#line 40
}
#line 40
# 102 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__clear(void )
#line 102
{
#line 102
  P1IFG &= ~(1 << 0);
}

# 52 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__clear(void ){
#line 52
  HplMsp430InterruptP__Port10__clear();
#line 52
}
#line 52
# 94 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__disable(void )
#line 94
{
#line 94
  P1IE &= ~(1 << 0);
}

# 47 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__disable(void ){
#line 47
  HplMsp430InterruptP__Port10__disable();
#line 47
}
#line 47
# 69 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__Interrupt__disable(void )
#line 69
{
  /* atomic removed: atomic calls only */
#line 70
  {
    /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__disable();
    /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__clear();
  }
  return SUCCESS;
}

# 61 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420XDriverLayerP__FifopInterrupt__disable(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__Interrupt__disable();
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 61 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )34U &= ~(0x01 << 0);
}

# 78 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__makeInput();
#line 78
}
#line 78
# 52 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__GeneralIO__makeInput(void )
#line 52
{
#line 52
  /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__makeInput();
}

# 44 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__FIFOP__makeInput(void ){
#line 44
  /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__GeneralIO__makeInput();
#line 44
}
#line 44
# 61 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )34U &= ~(0x01 << 3);
}

# 78 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.FIFOM*/Msp430GpioC__5__HplGeneralIO__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__makeInput();
#line 78
}
#line 78
# 52 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.FIFOM*/Msp430GpioC__5__GeneralIO__makeInput(void )
#line 52
{
#line 52
  /*HplCC2420XC.FIFOM*/Msp430GpioC__5__HplGeneralIO__makeInput();
}

# 44 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__FIFO__makeInput(void ){
#line 44
  /*HplCC2420XC.FIFOM*/Msp430GpioC__5__GeneralIO__makeInput();
#line 44
}
#line 44
# 61 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )30U &= ~(0x01 << 1);
}

# 78 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput();
#line 78
}
#line 78
# 52 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void )
#line 52
{
#line 52
  /*HplCC2420XC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput();
}

# 44 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__SFD__makeInput(void ){
#line 44
  /*HplCC2420XC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput();
#line 44
}
#line 44
# 61 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )34U &= ~(0x01 << 4);
}

# 78 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput();
#line 78
}
#line 78
# 52 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void )
#line 52
{
#line 52
  /*HplCC2420XC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput();
}

# 44 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__CCA__makeInput(void ){
#line 44
  /*HplCC2420XC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput();
#line 44
}
#line 44
# 63 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )30U |= 0x01 << 6;
}

# 85 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput();
#line 85
}
#line 85
# 54 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*HplCC2420XC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput();
}

# 46 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__RSTN__makeOutput(void ){
#line 46
  /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )30U |= 0x01 << 5;
}

# 85 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput();
#line 85
}
#line 85
# 54 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*HplCC2420XC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput();
}

# 46 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__VREN__makeOutput(void ){
#line 46
  /*HplCC2420XC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput();
#line 46
}
#line 46
# 339 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static inline error_t CC2420XDriverLayerP__SoftwareInit__init(void )
{

  CC2420XDriverLayerP__CSN__makeOutput();
  CC2420XDriverLayerP__VREN__makeOutput();
  CC2420XDriverLayerP__RSTN__makeOutput();
  CC2420XDriverLayerP__CCA__makeInput();
  CC2420XDriverLayerP__SFD__makeInput();
  CC2420XDriverLayerP__FIFO__makeInput();
  CC2420XDriverLayerP__FIFOP__makeInput();

  CC2420XDriverLayerP__FifopInterrupt__disable();
  CC2420XDriverLayerP__SfdCapture__disable();


  CC2420XDriverLayerP__CSN__set();


  CC2420XDriverLayerP__VREN__set();
  CC2420XDriverLayerP__BusyWait__wait(600);


  CC2420XDriverLayerP__RSTN__clr();
  CC2420XDriverLayerP__RSTN__set();

  CC2420XDriverLayerP__rxMsg = &CC2420XDriverLayerP__rxMsgBuffer;

  CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_VR_ON;



  return CC2420XDriverLayerP__SpiResource__request();
}

# 55 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void )
#line 55
{
  memset(/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY, sizeof /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ);
  return SUCCESS;
}

# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init();
#line 62
  __nesc_result = ecombine(__nesc_result, CC2420XDriverLayerP__SoftwareInit__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, RandomMlcgC__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__SoftwareInit__init());
#line 62
  __nesc_result = ecombine(__nesc_result, NeighborhoodP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*CC2420XRadioC.UniqueLayerC.UniqueLayerP*/UniqueLayerP__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*CC2420XRadioC.SendResourceC.Queue*/FcfsResourceQueueC__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init());
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 63 "BlinkToRadioC.nc"
static inline void BlinkToRadioC__Boot__booted(void )
#line 63
{
  BlinkToRadioC__AMControl__start();
}

# 60 "/opt/tinyos-2.1.2/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 60
  BlinkToRadioC__Boot__booted();
#line 60
}
#line 60
# 391 "/opt/tinyos-2.1.2/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
  __dint();
  __nop();
}

#line 379
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 379
{
  return m1 < m2 ? m1 : m2;
}

# 60 "/opt/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/Msp430ClockP.nc"
static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void )
#line 60
{
  return MSP430_POWER_LPM3;
}

# 140 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__areEventsEnabled(void )
{
  return * (volatile uint16_t * )390U & 0x0010;
}

# 59 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static bool /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__areEventsEnabled(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__areEventsEnabled();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 76 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline bool /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__isRunning(void )
{
  return /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__areEventsEnabled();
}

# 88 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static bool Msp430HybridAlarmCounterP__AlarmMicro__isRunning(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__isRunning();
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 260 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
static inline mcu_power_t Msp430HybridAlarmCounterP__McuPowerOverride__lowestState(void )
#line 260
{
  if (Msp430HybridAlarmCounterP__AlarmMicro__isRunning()) {
    return MSP430_POWER_LPM0;
    }
  else {
#line 264
    return MSP430_POWER_LPM3;
    }
}

# 62 "/opt/tinyos-2.1.2/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430HybridAlarmCounterP__McuPowerOverride__lowestState();
#line 62
  __nesc_result = mcombine(__nesc_result, Msp430ClockP__McuPowerOverride__lowestState());
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 79 "/opt/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/McuSleepC.nc"
static inline mcu_power_t McuSleepC__getPowerState(void )
#line 79
{
  mcu_power_t pState = MSP430_POWER_LPM4;









  if ((((((
#line 82
  TBCCTL0 & 0x0010 || 
  TBCCTL1 & 0x0010) || 
  TBCCTL2 & 0x0010) && (
  TBCTL & 0x0300) == 0x0200) || (
  ME1 & (0x80 | 0x40) && U0TCTL & 0x20)) || (
  ME2 & (0x20 | 0x10) && U1TCTL & 0x20))


   || (U0CTLnr & 0x01 && I2CTCTLnr & 0x20 && 
  I2CDCTLnr & 0x20 && U0CTLnr & 0x04 && U0CTLnr & 0x20)) {


    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x010) {
      if (ADC12CTL1 & 0x0010) {

          if (ADC12CTL1 & 0x0008) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 104
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 105
        if (ADC12CTL1 & 0x0400 && (TACTL & 0x0300) == 0x0200) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

static inline void McuSleepC__computePowerState(void )
#line 117
{
  McuSleepC__powerState = mcombine(McuSleepC__getPowerState(), 
  McuSleepC__McuPowerOverride__lowestState());
}

static inline void McuSleepC__McuSleep__sleep(void )
#line 122
{
  uint16_t temp;

#line 124
  if (McuSleepC__dirty) {
      McuSleepC__computePowerState();
    }

  temp = McuSleepC__msp430PowerBits[McuSleepC__powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 76 "/opt/tinyos-2.1.2/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP__McuSleep__sleep(void ){
#line 76
  McuSleepC__McuSleep__sleep();
#line 76
}
#line 76
# 78 "/opt/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP__popTask(void )
{
  if (SchedulerBasicP__m_head != SchedulerBasicP__NO_TASK) 
    {
      uint8_t id = SchedulerBasicP__m_head;

#line 83
      SchedulerBasicP__m_head = SchedulerBasicP__m_next[SchedulerBasicP__m_head];
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
        }
      SchedulerBasicP__m_next[id] = SchedulerBasicP__NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP__NO_TASK;
    }
}

#line 149
static inline void SchedulerBasicP__Scheduler__taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP__popTask()) == SchedulerBasicP__NO_TASK) 
            {
              SchedulerBasicP__McuSleep__sleep();
            }
        }
#line 161
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP__TaskBasic__runTask(nextTask);
    }
}

# 72 "/opt/tinyos-2.1.2/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__taskLoop(void ){
#line 72
  SchedulerBasicP__Scheduler__taskLoop();
#line 72
}
#line 72
# 98 "/opt/tinyos-2.1.2/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void ){
#line 98
  unsigned char __nesc_result;
#line 98

#line 98
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 349 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__disableRxIntr(void )
#line 349
{
  HplMsp430Usart0P__IE1 &= ~0x40;
}

# 177 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr(void ){
#line 177
  HplMsp430Usart0P__Usart__disableRxIntr();
#line 177
}
#line 177
# 182 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp(void )
#line 182
{

  uint8_t end;
  uint8_t tmp;

  /* atomic removed: atomic calls only */
#line 187
  {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf ? /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos] : 0);

    end = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos + /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SPI_ATOMIC_SIZE;
    if (end > /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len) {
      end = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len;
      }
    while (++/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos < end) {
        while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending()) ;
        tmp = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx();
        if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf) {
          /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos - 1] = tmp;
          }
#line 199
        /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf ? /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos] : 0);
      }
  }
}

#line 231
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data)
#line 231
{

  if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf) {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos - 1] = data;
    }
  if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos < /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len) {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp();
    }
  else 
#line 238
    {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr();
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone();
    }
}

# 65 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(uint8_t arg_0x2b395b0a50c8, uint8_t data){
#line 54
  switch (arg_0x2b395b0a50c8) {
#line 54
    case /*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 54
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(data);
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(arg_0x2b395b0a50c8, data);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 90 "/opt/tinyos-2.1.2/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 54 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(), data);
    }
}

# 54 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data){
#line 54
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(data);
#line 54
}
#line 54
# 55 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static inline bool HplMsp430I2C0P__HplI2C__isI2C(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 56
  {
    unsigned char __nesc_temp = 
#line 56
    HplMsp430I2C0P__U0CTL & 0x20 && HplMsp430I2C0P__U0CTL & 0x04 && HplMsp430I2C0P__U0CTL & 0x01;

#line 56
    return __nesc_temp;
  }
}

# 6 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static bool HplMsp430Usart0P__HplI2C__isI2C(void ){
#line 6
  unsigned char __nesc_result;
#line 6

#line 6
  __nesc_result = HplMsp430I2C0P__HplI2C__isI2C();
#line 6

#line 6
  return __nesc_result;
#line 6
}
#line 6
# 66 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(uint8_t id)
#line 66
{
}

# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(uint8_t arg_0x2b395b0a4020){
#line 39
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(arg_0x2b395b0a4020);
#line 39
}
#line 39
# 59 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void )
#line 59
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void HplMsp430Usart0P__I2CInterrupts__fired(void ){
#line 39
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired();
#line 39
}
#line 39
# 249 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void )
#line 249
{
}

# 64 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id)
#line 64
{
}

# 49 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(uint8_t arg_0x2b395b0a50c8){
#line 49
  switch (arg_0x2b395b0a50c8) {
#line 49
    case /*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 49
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone();
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(arg_0x2b395b0a50c8);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 49 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 49 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__txDone(void ){
#line 49
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone();
#line 49
}
#line 49
# 1030 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static inline void CC2420XDriverLayerP__FifopInterrupt__fired(void )
{
}

# 68 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__Interrupt__fired(void ){
#line 68
  CC2420XDriverLayerP__FifopInterrupt__fired();
#line 68
}
#line 68
# 77 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__fired(void )
#line 77
{
  /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__clear();
  /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__Interrupt__fired();
}

# 72 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port10__fired(void ){
#line 72
  /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__fired();
#line 72
}
#line 72
# 103 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port11__clear(void )
#line 103
{
#line 103
  P1IFG &= ~(1 << 1);
}

#line 79
static inline void HplMsp430InterruptP__Port11__default__fired(void )
#line 79
{
#line 79
  HplMsp430InterruptP__Port11__clear();
}

# 72 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port11__fired(void ){
#line 72
  HplMsp430InterruptP__Port11__default__fired();
#line 72
}
#line 72
# 104 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port12__clear(void )
#line 104
{
#line 104
  P1IFG &= ~(1 << 2);
}

#line 80
static inline void HplMsp430InterruptP__Port12__default__fired(void )
#line 80
{
#line 80
  HplMsp430InterruptP__Port12__clear();
}

# 72 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port12__fired(void ){
#line 72
  HplMsp430InterruptP__Port12__default__fired();
#line 72
}
#line 72
# 105 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port13__clear(void )
#line 105
{
#line 105
  P1IFG &= ~(1 << 3);
}

#line 81
static inline void HplMsp430InterruptP__Port13__default__fired(void )
#line 81
{
#line 81
  HplMsp430InterruptP__Port13__clear();
}

# 72 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port13__fired(void ){
#line 72
  HplMsp430InterruptP__Port13__default__fired();
#line 72
}
#line 72
# 106 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__clear(void )
#line 106
{
#line 106
  P1IFG &= ~(1 << 4);
}

#line 82
static inline void HplMsp430InterruptP__Port14__default__fired(void )
#line 82
{
#line 82
  HplMsp430InterruptP__Port14__clear();
}

# 72 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port14__fired(void ){
#line 72
  HplMsp430InterruptP__Port14__default__fired();
#line 72
}
#line 72
# 107 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port15__clear(void )
#line 107
{
#line 107
  P1IFG &= ~(1 << 5);
}

#line 83
static inline void HplMsp430InterruptP__Port15__default__fired(void )
#line 83
{
#line 83
  HplMsp430InterruptP__Port15__clear();
}

# 72 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port15__fired(void ){
#line 72
  HplMsp430InterruptP__Port15__default__fired();
#line 72
}
#line 72
# 108 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port16__clear(void )
#line 108
{
#line 108
  P1IFG &= ~(1 << 6);
}

#line 84
static inline void HplMsp430InterruptP__Port16__default__fired(void )
#line 84
{
#line 84
  HplMsp430InterruptP__Port16__clear();
}

# 72 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port16__fired(void ){
#line 72
  HplMsp430InterruptP__Port16__default__fired();
#line 72
}
#line 72
# 109 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port17__clear(void )
#line 109
{
#line 109
  P1IFG &= ~(1 << 7);
}

#line 85
static inline void HplMsp430InterruptP__Port17__default__fired(void )
#line 85
{
#line 85
  HplMsp430InterruptP__Port17__clear();
}

# 72 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port17__fired(void ){
#line 72
  HplMsp430InterruptP__Port17__default__fired();
#line 72
}
#line 72
# 206 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port20__clear(void )
#line 206
{
#line 206
  P2IFG &= ~(1 << 0);
}

#line 182
static inline void HplMsp430InterruptP__Port20__default__fired(void )
#line 182
{
#line 182
  HplMsp430InterruptP__Port20__clear();
}

# 72 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port20__fired(void ){
#line 72
  HplMsp430InterruptP__Port20__default__fired();
#line 72
}
#line 72
# 207 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port21__clear(void )
#line 207
{
#line 207
  P2IFG &= ~(1 << 1);
}

#line 183
static inline void HplMsp430InterruptP__Port21__default__fired(void )
#line 183
{
#line 183
  HplMsp430InterruptP__Port21__clear();
}

# 72 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port21__fired(void ){
#line 72
  HplMsp430InterruptP__Port21__default__fired();
#line 72
}
#line 72
# 208 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port22__clear(void )
#line 208
{
#line 208
  P2IFG &= ~(1 << 2);
}

#line 184
static inline void HplMsp430InterruptP__Port22__default__fired(void )
#line 184
{
#line 184
  HplMsp430InterruptP__Port22__clear();
}

# 72 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port22__fired(void ){
#line 72
  HplMsp430InterruptP__Port22__default__fired();
#line 72
}
#line 72
# 209 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port23__clear(void )
#line 209
{
#line 209
  P2IFG &= ~(1 << 3);
}

#line 185
static inline void HplMsp430InterruptP__Port23__default__fired(void )
#line 185
{
#line 185
  HplMsp430InterruptP__Port23__clear();
}

# 72 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port23__fired(void ){
#line 72
  HplMsp430InterruptP__Port23__default__fired();
#line 72
}
#line 72
# 210 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port24__clear(void )
#line 210
{
#line 210
  P2IFG &= ~(1 << 4);
}

#line 186
static inline void HplMsp430InterruptP__Port24__default__fired(void )
#line 186
{
#line 186
  HplMsp430InterruptP__Port24__clear();
}

# 72 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port24__fired(void ){
#line 72
  HplMsp430InterruptP__Port24__default__fired();
#line 72
}
#line 72
# 211 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port25__clear(void )
#line 211
{
#line 211
  P2IFG &= ~(1 << 5);
}

#line 187
static inline void HplMsp430InterruptP__Port25__default__fired(void )
#line 187
{
#line 187
  HplMsp430InterruptP__Port25__clear();
}

# 72 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port25__fired(void ){
#line 72
  HplMsp430InterruptP__Port25__default__fired();
#line 72
}
#line 72
# 212 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port26__clear(void )
#line 212
{
#line 212
  P2IFG &= ~(1 << 6);
}

#line 188
static inline void HplMsp430InterruptP__Port26__default__fired(void )
#line 188
{
#line 188
  HplMsp430InterruptP__Port26__clear();
}

# 72 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port26__fired(void ){
#line 72
  HplMsp430InterruptP__Port26__default__fired();
#line 72
}
#line 72
# 213 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port27__clear(void )
#line 213
{
#line 213
  P2IFG &= ~(1 << 7);
}

#line 189
static inline void HplMsp430InterruptP__Port27__default__fired(void )
#line 189
{
#line 189
  HplMsp430InterruptP__Port27__clear();
}

# 72 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port27__fired(void ){
#line 72
  HplMsp430InterruptP__Port27__default__fired();
#line 72
}
#line 72
# 411 "/opt/tinyos-2.1.2/tos/chips/msp430/msp430hardware.h"
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (__read_status_register() & 0x0008) != 0;

#line 414
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}

  void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
   __asm volatile ("" :  :  : "memory");
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 11 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x000C)))  void sig_TIMERA0_VECTOR(void )
#line 11
{
#line 11
  Msp430TimerCommonP__VectorTimerA0__fired();
}

# 48 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
#line 48
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow();
#line 48
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow();
#line 48
  /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow();
#line 48
}
#line 48
# 180 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired();
    }
}

# 170 "/opt/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 172
    {
#line 172
      {
        unsigned char __nesc_temp = 
#line 172
        SchedulerBasicP__pushTask(id) ? SUCCESS : EBUSY;

        {
#line 172
          __nesc_atomic_end(__nesc_atomic); 
#line 172
          return __nesc_temp;
        }
      }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
}

# 107 "/opt/tinyos-2.1.2/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
#line 109
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type expires;
#line 109
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 132
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )remaining << 5);
}

# 80 "/opt/tinyos-2.1.2/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type rv = 0;

#line 83
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();

#line 87
      if (/*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT;

#line 101
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC__0__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 62 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void )
{




  if (1) {
      /* atomic removed: atomic calls only */
#line 69
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )368U;

#line 72
        do {
#line 72
            t0 = t1;
#line 72
            t1 = * (volatile uint16_t * )368U;
          }
        while (
#line 72
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 73
          t1;

#line 73
          return __nesc_temp;
        }
      }
    }
  else 
#line 76
    {
      return * (volatile uint16_t * )368U;
    }
}

# 180 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired();
    }
}

#line 180
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired();
    }
}

# 12 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x000A)))  void sig_TIMERA1_VECTOR(void )
#line 12
{
#line 12
  Msp430TimerCommonP__VectorTimerA1__fired();
}

#line 13
__attribute((wakeup)) __attribute((interrupt(0x001A)))  void sig_TIMERB0_VECTOR(void )
#line 13
{
#line 13
  Msp430TimerCommonP__VectorTimerB0__fired();
}

# 146 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n)
{
}

# 39 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x2b395a188458){
#line 39
  switch (arg_0x2b395a188458) {
#line 39
    case 0:
#line 39
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired();
#line 39
      break;
#line 39
    case 1:
#line 39
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired();
#line 39
      break;
#line 39
    case 2:
#line 39
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired();
#line 39
      break;
#line 39
    case 3:
#line 39
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired();
#line 39
      break;
#line 39
    case 4:
#line 39
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired();
#line 39
      break;
#line 39
    case 5:
#line 39
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired();
#line 39
      break;
#line 39
    case 6:
#line 39
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired();
#line 39
      break;
#line 39
    case 7:
#line 39
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired();
#line 39
      break;
#line 39
    default:
#line 39
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x2b395a188458);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 112 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/TaskletC.nc"
static void TaskletC__Tasklet__schedule(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if (TaskletC__state != 0) 
        {
          TaskletC__state |= 0x80;
          {
#line 119
            __nesc_atomic_end(__nesc_atomic); 
#line 119
            return;
          }
        }
      TaskletC__state = 1;
    }
#line 123
    __nesc_atomic_end(__nesc_atomic); }

  TaskletC__doit();
}

#line 74
static void TaskletC__doit(void )
{
  for (; ; ) 
    {
      TaskletC__Tasklet__run();

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          if (TaskletC__state == 1) 
            {
              TaskletC__state = 0;
              {
#line 85
                __nesc_atomic_end(__nesc_atomic); 
#line 85
                return;
              }
            }
          for (; 0; ) ;
          TaskletC__state = 1;
        }
#line 90
        __nesc_atomic_end(__nesc_atomic); }
    }
}

# 423 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static bool CC2420XDriverLayerP__isSpiAcquired(void )
{
  if (CC2420XDriverLayerP__SpiResource__isOwner()) {
    return TRUE;
    }
  if (CC2420XDriverLayerP__SpiResource__immediateRequest() == SUCCESS) 
    {
      CC2420XDriverLayerP__CSN__makeOutput();
      CC2420XDriverLayerP__CSN__set();

      return TRUE;
    }

  CC2420XDriverLayerP__SpiResource__request();
  return FALSE;
}

# 133 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void )
#line 133
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 134
    {
      if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id) {
          if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING) {
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
              {
                unsigned char __nesc_temp = 
#line 138
                SUCCESS;

                {
#line 138
                  __nesc_atomic_end(__nesc_atomic); 
#line 138
                  return __nesc_temp;
                }
              }
            }
          else {
#line 140
            if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING) {
                /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
                /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
                {
                  unsigned char __nesc_temp = 
#line 143
                  SUCCESS;

                  {
#line 143
                    __nesc_atomic_end(__nesc_atomic); 
#line 143
                    return __nesc_temp;
                  }
                }
              }
            }
        }
    }
#line 149
    __nesc_atomic_end(__nesc_atomic); }
#line 147
  return FAIL;
}

# 265 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config)
#line 265
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 267
    {
      HplMsp430Usart0P__Usart__resetUsart(TRUE);
      HplMsp430Usart0P__HplI2C__clearModeI2C();
      HplMsp430Usart0P__Usart__disableUart();
      HplMsp430Usart0P__configSpi(config);
      HplMsp430Usart0P__Usart__enableSpi();
      HplMsp430Usart0P__Usart__resetUsart(FALSE);
      HplMsp430Usart0P__Usart__clrIntr();
      HplMsp430Usart0P__Usart__disableIntr();
    }
#line 276
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 63 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void )
#line 63
{
#line 63
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 63
    * (volatile uint8_t * )30U |= 0x01 << 2;
#line 63
    __nesc_atomic_end(__nesc_atomic); }
}

#line 56
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void )
#line 56
{
#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    * (volatile uint8_t * )29U |= 0x01 << 2;
#line 56
    __nesc_atomic_end(__nesc_atomic); }
}

# 77 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(uint8_t id)
#line 77
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 79
    {
      if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) {
          /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING;
          /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = id;
        }
      else {
#line 84
        if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId == id) {
            {
              unsigned char __nesc_temp = 
#line 85
              SUCCESS;

              {
#line 85
                __nesc_atomic_end(__nesc_atomic); 
#line 85
                return __nesc_temp;
              }
            }
          }
        else 
#line 87
          {
            unsigned char __nesc_temp = 
#line 87
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(id);

            {
#line 87
              __nesc_atomic_end(__nesc_atomic); 
#line 87
              return __nesc_temp;
            }
          }
        }
    }
#line 91
    __nesc_atomic_end(__nesc_atomic); }
#line 89
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested();
  return SUCCESS;
}

# 49 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/GpioCaptureC.nc"
static error_t /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__enableCapture(uint8_t mode)
#line 49
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 50
    {
      /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__disableEvents();
      /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__GeneralIO__selectModuleFunc();
      /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt();
      /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430Capture__clearOverflow();
      /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(mode);
      /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__enableEvents();
    }
#line 57
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 57 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void )
#line 57
{
#line 57
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    * (volatile uint8_t * )29U &= ~(0x01 << 2);
#line 57
    __nesc_atomic_end(__nesc_atomic); }
}

# 97 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckLayerC.nc"
static void /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__SubSend__sendDone(error_t error)
{
  if (/*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__state == /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__STATE_ACK_SEND) 
    {

      for (; 0; ) ;

      /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__state = /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__STATE_READY;
    }
  else 
    {
      for (; 0; ) ;
      for (; 0; ) ;

      if (error == SUCCESS && /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__requiresAckWait(/*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__txMsg) && /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioAlarm__isFree()) 
        {
          /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioAlarm__wait(/*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__Config__getAckTimeout());
          /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__state = /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__STATE_ACK_WAIT;
        }
      else 
        {
          /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__state = /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__STATE_READY;
          /*CC2420XRadioC.SoftwareAckLayerC*/SoftwareAckLayerC__0__RadioSend__sendDone(error);
        }
    }
}

# 132 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getAckRequired(message_t *msg)
{
  return __nesc_ntoh_leuint16(/*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->fcf.nxdata) & (1 << IEEE154_FCF_ACK_REQ) ? TRUE : FALSE;
}

#line 92
static bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isDataFrame(message_t *msg)
{
  return (__nesc_ntoh_leuint16(/*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->fcf.nxdata) & /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_DATA_FRAME_MASK) == /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_DATA_FRAME_VALUE;
}

#line 178
static uint16_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDestAddr(message_t *msg)
{
  return __nesc_ntoh_leuint16(/*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->dest.nxdata);
}

# 81 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      uint16_t now = /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get();
      uint16_t elapsed = now - t0;

#line 87
      if (elapsed >= dt) 
        {
          /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
        }
      else 
        {
          uint16_t remaining = dt - elapsed;

#line 94
          if (remaining <= 2) {
            /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
            }
          else {
#line 97
            /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(now + remaining);
            }
        }
#line 99
      /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt();
      /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents();
    }
#line 101
    __nesc_atomic_end(__nesc_atomic); }
}

# 217 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MessageBufferLayerP.nc"
static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioSend__sendDone(error_t error)
{
  for (; 0; ) ;

  /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__txError = error;
  if (error == SUCCESS) {
    /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__state = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__STATE_TX_DONE;
    }
  else {
#line 225
    /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__state = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__STATE_TX_PENDING;
    }
  /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__sendTask__postTask();
}

# 69 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/GpioCaptureC.nc"
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__disable(void )
#line 69
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 70
    {
      /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__disableEvents();
      /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__GeneralIO__selectIOFunc();
    }
#line 73
    __nesc_atomic_end(__nesc_atomic); }
}

# 57 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void )
#line 57
{
#line 57
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    * (volatile uint8_t * )29U &= ~(0x01 << 6);
#line 57
    __nesc_atomic_end(__nesc_atomic); }
}

#line 56
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void )
#line 56
{
#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    * (volatile uint8_t * )29U |= 0x01 << 6;
#line 56
    __nesc_atomic_end(__nesc_atomic); }
}

# 63 "/opt/tinyos-2.1.2/tos/lib/timer/BusyWaitCounterC.nc"
static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {


      /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type t0 = /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get();

      if (dt > /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__HALF_MAX_SIZE_TYPE) 
        {
          dt -= /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__HALF_MAX_SIZE_TYPE;
          while (/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get() - t0 <= dt) ;
          t0 += dt;
          dt = /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__HALF_MAX_SIZE_TYPE;
        }

      while (/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get() - t0 <= dt) ;
    }
#line 80
    __nesc_atomic_end(__nesc_atomic); }
}

# 104 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isAckFrame(message_t *msg)
{
  return (__nesc_ntoh_leuint16(/*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->fcf.nxdata) & /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_ACK_FRAME_MASK) == /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_ACK_FRAME_VALUE;
}

# 66 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MetadataFlagsLayerC.nc"
static void /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__PacketFlag__set(uint8_t bit, message_t *msg)
{
  for (; 0; ) ;

  /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__getMeta(msg)->flags |= 1 << bit;
}

# 95 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static cc2420x_metadata_t *CC2420XDriverLayerP__getMeta(message_t *msg)
{
  return (void *)msg + sizeof(message_t ) - CC2420XDriverLayerP__RadioPacket__metadataLength(msg);
}

# 80 "/opt/tinyos-2.1.2/tos/lib/timer/TransformCounterC.nc"
static /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_size_type /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__get(void )
{
  /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_size_type rv = 0;

  /* atomic removed: atomic calls only */
#line 84
  {
    /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__upper_count_type high = /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__m_upper;
    /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__from_size_type low = /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__get();

#line 87
    if (/*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__isOverflowPending()) 
      {






        high++;
        low = /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__get();
      }
    {
      /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_size_type high_to = high;
      /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_size_type low_to = low >> /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__LOW_SHIFT_RIGHT;

#line 101
      rv = (high_to << /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__HIGH_SHIFT_LEFT) | low_to;
    }
  }
  return rv;
}

# 65 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
static uint8_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__SubPacket__metadataLength(message_t *msg){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__RadioPacket__metadataLength(msg);
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 205 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static bool /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__requiresAckReply(message_t *msg)
{
  return /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getAckRequired(msg)
   && /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isDataFrame(msg)
   && /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDestAddr(msg) == /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__ActiveMessageAddress__amAddress();
}

# 106 "/opt/tinyos-2.1.2/tos/system/ActiveMessageAddressC.nc"
static am_addr_t ActiveMessageAddressC__amAddress(void )
#line 106
{
  am_addr_t myAddr;

#line 108
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 108
    myAddr = ActiveMessageAddressC__addr;
#line 108
    __nesc_atomic_end(__nesc_atomic); }
  return myAddr;
}

# 1191 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static void CC2420XDriverLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length)
{
  for (; 0; ) ;
  for (; 0; ) ;


  __nesc_hton_leuint8(CC2420XDriverLayerP__getHeader(msg)->length.nxdata, length + 2);
}

#line 611
static error_t CC2420XDriverLayerP__RadioSend__send(message_t *msg)
{
  uint16_t time;
  uint8_t p;
  uint8_t length;
  uint8_t *data;
  uint8_t header;
  uint32_t time32;
  void *timesync;
  timesync_relative_t timesync_relative;
  uint32_t sfdTime;

  if (((CC2420XDriverLayerP__cmd != CC2420XDriverLayerP__CMD_NONE || (CC2420XDriverLayerP__state != CC2420XDriverLayerP__STATE_IDLE && CC2420XDriverLayerP__state != CC2420XDriverLayerP__STATE_RX_ON)) || !CC2420XDriverLayerP__isSpiAcquired()) || CC2420XDriverLayerP__radioIrq) {
    return EBUSY;
    }
  p = (CC2420XDriverLayerP__PacketTransmitPower__isSet(msg) ? 
  CC2420XDriverLayerP__PacketTransmitPower__get(msg) : 31) & CC2420X_TX_PWR_MASK;

  if (p != CC2420XDriverLayerP__txPower) 
    {
      cc2420X_txctrl_t txctrl = cc2420X_txctrl_default;

      CC2420XDriverLayerP__txPower = p;

      txctrl.f.pa_level = CC2420XDriverLayerP__txPower;
      CC2420XDriverLayerP__writeRegister(CC2420X_TXCTRL, txctrl.value);
    }

  if (CC2420XDriverLayerP__Config__requiresRssiCca(msg) && !CC2420XDriverLayerP__CCA__get()) {
    return EBUSY;
    }
  data = CC2420XDriverLayerP__getPayload(msg);
  length = __nesc_ntoh_leuint8(CC2420XDriverLayerP__getHeader(msg)->length.nxdata);



  header = CC2420XDriverLayerP__Config__headerPreloadLength();
  if (header > length) {
    header = length;
    }
  length -= header;


  CC2420XDriverLayerP__SfdCapture__disable();


  CC2420XDriverLayerP__writeTxFifo(data, header);




  if (((CC2420XDriverLayerP__cmd != CC2420XDriverLayerP__CMD_NONE || (CC2420XDriverLayerP__state != CC2420XDriverLayerP__STATE_IDLE && CC2420XDriverLayerP__state != CC2420XDriverLayerP__STATE_RX_ON)) || CC2420XDriverLayerP__radioIrq) || CC2420XDriverLayerP__SFD__get() == 1) {

      CC2420XDriverLayerP__strobe(CC2420X_SFLUSHTX);

      CC2420XDriverLayerP__SfdCapture__captureRisingEdge();

      return EBUSY;
    }




  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 674
    {


      CC2420XDriverLayerP__capturedTime = 0;


      CC2420XDriverLayerP__strobe(CC2420X_STXON);


      time = CC2420XDriverLayerP__RadioAlarm__getNow();

      CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_TRANSMIT;
      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_TX_ON;
      CC2420XDriverLayerP__SfdCapture__captureFallingEdge();
    }
#line 688
    __nesc_atomic_end(__nesc_atomic); }

  timesync = CC2420XDriverLayerP__PacketTimeSyncOffset__isSet(msg) ? (void *)msg + CC2420XDriverLayerP__PacketTimeSyncOffset__get(msg) : 0;

  if (timesync == 0) {

      if (length > 0) {
        CC2420XDriverLayerP__writeTxFifo(data + header, length - 1);
        }
#line 696
      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_BUSY_TX_2_RX_ON;
    }
  else 
#line 697
    {


      CC2420XDriverLayerP__writeTxFifo(data + header, length - sizeof timesync_relative - 1);
    }



  sfdTime = time;



  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 709
    {
      time = CC2420XDriverLayerP__RadioAlarm__getNow();
      time32 = CC2420XDriverLayerP__LocalTime__get();
    }
#line 712
    __nesc_atomic_end(__nesc_atomic); }


  time -= sfdTime;
  time32 -= time;


  time32 += TX_SFD_DELAY;

  CC2420XDriverLayerP__PacketTimeStamp__set(msg, time32);

  if (timesync != 0) {

      __nesc_hton_int32(timesync_relative.nxdata, __nesc_ntoh_uint32((* (timesync_absolute_t *)timesync).nxdata) - time32);



      CC2420XDriverLayerP__writeTxFifo((uint8_t *)&timesync_relative, sizeof timesync_relative);
      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_BUSY_TX_2_RX_ON;
    }
#line 749
  return SUCCESS;
}

# 61 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MetadataFlagsLayerC.nc"
static bool /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__PacketFlag__get(uint8_t bit, message_t *msg)
{
  return /*CC2420XRadioC.MetadataFlagsLayerC*/MetadataFlagsLayerC__0__getMeta(msg)->flags & (1 << bit);
}

# 1186 "/opt/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverLayerP.nc"
static uint8_t CC2420XDriverLayerP__RadioPacket__payloadLength(message_t *msg)
{
  return __nesc_ntoh_leuint8(CC2420XDriverLayerP__getHeader(msg)->length.nxdata) - 2;
}

# 50 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__RadioAlarm__wait(tradio_size timeout){
#line 50
  /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__wait(1U, timeout);
#line 50
}
#line 50
static void CC2420XDriverLayerP__RadioAlarm__wait(tradio_size timeout){
#line 50
  /*CC2420XRadioC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__wait(3U, timeout);
#line 50
}
#line 50
# 252 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MessageBufferLayerP.nc"
static void /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioSend__ready(void )
{
  if (/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__state == /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__STATE_TX_RETRY) 
    {
      /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__state = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__STATE_TX_PENDING;
      /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__sendTask__postTask();
    }
}

# 84 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/RandomCollisionLayerP.nc"
static uint16_t /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__getBackoff(uint16_t maxBackoff)
{
  uint16_t a;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      a = /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__nextRandom;
      /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__nextRandom += 273;
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__calcNextRandom__postTask();

  return a % maxBackoff + /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__Config__getMinimumBackoff();
}

# 14 "/opt/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0018)))  void sig_TIMERB1_VECTOR(void )
#line 14
{
#line 14
  Msp430TimerCommonP__VectorTimerB1__fired();
}

# 63 "/opt/tinyos-2.1.2/tos/system/RealMainP.nc"
  int main(void )
#line 63
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 71
      ;

      RealMainP__Scheduler__init();





      RealMainP__PlatformInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;





      RealMainP__SoftwareInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;
    }
#line 88
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP__Boot__booted();


  RealMainP__Scheduler__taskLoop();




  return -1;
}

# 172 "/opt/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/Msp430ClockP.nc"
static void Msp430ClockP__set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 16 "/opt/tinyos-2.1.2/tos/platforms/telosb/MotePlatformC.nc"
static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set)
#line 16
{
  if (set) {
    TOSH_SET_SIMO0_PIN();
    }
  else {
#line 20
    TOSH_CLR_SIMO0_PIN();
    }
#line 21
  TOSH_SET_UCLK0_PIN();
  TOSH_CLR_UCLK0_PIN();
}

# 134 "/opt/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP__Scheduler__runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 138
  {
    nextTask = SchedulerBasicP__popTask();
    if (nextTask == SchedulerBasicP__NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 142
          FALSE;

#line 142
          return __nesc_temp;
        }
      }
  }
#line 145
  SchedulerBasicP__TaskBasic__runTask(nextTask);
  return TRUE;
}

#line 175
static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
{
}

# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x2b395a090650){
#line 75
  switch (arg_0x2b395a090650) {
#line 75
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired:
#line 75
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask();
#line 75
      break;
#line 75
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer:
#line 75
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask();
#line 75
      break;
#line 75
    case /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask:
#line 75
      /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__runTask();
#line 75
      break;
#line 75
    case /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__stateDoneTask:
#line 75
      /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__stateDoneTask__runTask();
#line 75
      break;
#line 75
    case /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__sendTask:
#line 75
      /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__sendTask__runTask();
#line 75
      break;
#line 75
    case /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__deliverTask:
#line 75
      /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__deliverTask__runTask();
#line 75
      break;
#line 75
    case /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__calcNextRandom:
#line 75
      /*CC2420XRadioC.CollisionAvoidanceLayerC.RandomCollisionLayerP*/RandomCollisionLayerP__0__calcNextRandom__runTask();
#line 75
      break;
#line 75
    case CC2420XDriverLayerP__releaseSpi:
#line 75
      CC2420XDriverLayerP__releaseSpi__runTask();
#line 75
      break;
#line 75
    case /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task:
#line 75
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask();
#line 75
      break;
#line 75
    case /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask:
#line 75
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask();
#line 75
      break;
#line 75
    case /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask:
#line 75
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask();
#line 75
      break;
#line 75
    case /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask:
#line 75
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask();
#line 75
      break;
#line 75
    default:
#line 75
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x2b395a090650);
#line 75
      break;
#line 75
    }
#line 75
}
#line 75
# 86 "BlinkToRadioC.nc"
static void BlinkToRadioC__AMSend__sendDone(message_t *msg, error_t err)
#line 86
{
  if (BlinkToRadioC__pktptr == msg) {
      BlinkToRadioC__busy = FALSE;
      BlinkToRadioC__resendTS = BlinkToRadioC__PacketTimeStamp__timestamp(msg);
    }
}

# 72 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TimeStampingLayerP.nc"
static uint32_t /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__PacketTimeStampRadio__timestamp(message_t *msg)
{
  return /*CC2420XRadioC.TimeStampingLayerC.TimeStampingLayerP*/TimeStampingLayerP__0__getMeta(msg)->timestamp;
}

# 163 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err)
#line 163
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend();
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, err);
}

# 282 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static uint8_t /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__RadioPacket__headerLength(message_t *msg)
{
  return /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__headerLength(msg) + sizeof(ieee154_simple_header_t );
}

# 79 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayerP.nc"
static error_t /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMSend__send(am_id_t id, am_addr_t addr, message_t *msg, uint8_t len)
{
  if (len > /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Packet__maxPayloadLength()) {
    return EINVAL;
    }
  if (/*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Config__checkFrame(msg) != SUCCESS) {
    return FAIL;
    }
  /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__Packet__setPayloadLength(msg, len);
  /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__setSource(msg, /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__address());
  /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__setGroup(msg, /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__localGroup());
  /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__setType(msg, id);
  /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__AMPacket__setDestination(msg, addr);

  /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SendNotifier__aboutToSend(id, addr, msg);

  return /*CC2420XRadioC.ActiveMessageLayerC.ActiveMessageLayerP*/ActiveMessageLayerP__0__SubSend__send(msg);
}

# 292 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static void /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__RadioPacket__setPayloadLength(message_t *msg, uint8_t length)
{
  /*CC2420XRadioC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__setPayloadLength(msg, length + sizeof(ieee154_simple_header_t ));
}

# 93 "/opt/tinyos-2.1.2/tos/system/ActiveMessageAddressC.nc"
static am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void )
#line 93
{
  am_group_t myGroup;

#line 95
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 95
    myGroup = ActiveMessageAddressC__group;
#line 95
    __nesc_atomic_end(__nesc_atomic); }
  return myGroup;
}

# 132 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/TinyosNetworkLayerC.nc"
static error_t /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__TinyosSend__send(message_t *msg)
{

  __nesc_hton_leuint8(/*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__getHeader(msg)->network.nxdata, 0x3f);

  return /*CC2420XRadioC.TinyosNetworkLayerC*/TinyosNetworkLayerC__0__SubSend__send(msg);
}

# 99 "/opt/tinyos-2.1.2/tos/lib/rfxlink/util/TaskletC.nc"
static void TaskletC__Tasklet__resume(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if (--TaskletC__state != 0x80) {
        {
#line 104
          __nesc_atomic_end(__nesc_atomic); 
#line 104
          return;
        }
        }
#line 106
      TaskletC__state = 1;
    }
#line 107
    __nesc_atomic_end(__nesc_atomic); }

  TaskletC__doit();
}

# 97 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static error_t /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__release(uint8_t id)
#line 97
{
  bool released = FALSE;

#line 99
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 99
    {
      if (/*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__state == /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_BUSY && /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId == id) {
          if (/*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__isEmpty() == FALSE) {
              /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId = /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__NO_RES;
              /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__reqResId = /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__dequeue();
              /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__state = /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_GRANTING;
              /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__postTask();
            }
          else {
              /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId = /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__NO_RES;
              /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__state = /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_IDLE;
            }
          released = TRUE;
        }
    }
#line 113
    __nesc_atomic_end(__nesc_atomic); }
  if (released == TRUE) {
      /*CC2420XRadioC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__unconfigure(id);
      return SUCCESS;
    }
  return FAIL;
}

# 111 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id)
#line 111
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    {
      if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY && /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
          if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty() == FALSE) {
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue();
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
            }
          else {
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted();
            }
          {
            unsigned char __nesc_temp = 
#line 127
            SUCCESS;

            {
#line 127
              __nesc_atomic_end(__nesc_atomic); 
#line 127
              return __nesc_temp;
            }
          }
        }
    }
#line 131
    __nesc_atomic_end(__nesc_atomic); }
#line 130
  return FAIL;
}

# 247 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static void HplMsp430Usart0P__Usart__disableSpi(void )
#line 247
{
  /* atomic removed: atomic calls only */
#line 248
  {
    HplMsp430Usart0P__ME1 &= ~0x40;
    HplMsp430Usart0P__SIMO__selectIOFunc();
    HplMsp430Usart0P__SOMI__selectIOFunc();
    HplMsp430Usart0P__UCLK__selectIOFunc();
  }
}

# 69 "/opt/tinyos-2.1.2/tos/system/RandomMlcgC.nc"
static uint32_t RandomMlcgC__Random__rand32(void )
#line 69
{
  uint32_t mlcg;
#line 70
  uint32_t p;
#line 70
  uint32_t q;
  uint64_t tmpseed;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      tmpseed = (uint64_t )33614U * (uint64_t )RandomMlcgC__seed;
      q = tmpseed;
      q = q >> 1;
      p = tmpseed >> 32;
      mlcg = p + q;
      if (mlcg & 0x80000000) {
          mlcg = mlcg & 0x7FFFFFFF;
          mlcg++;
        }
      RandomMlcgC__seed = mlcg;
    }
#line 84
    __nesc_atomic_end(__nesc_atomic); }
  return mlcg;
}

# 76 "/opt/tinyos-2.1.2/tos/lib/rfxlink/layers/MessageBufferLayerP.nc"
static error_t /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__SplitControl__start(void )
{
  error_t error;

  /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Tasklet__suspend();

  if (/*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__state != /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__STATE_READY) {
    error = EBUSY;
    }
  else {
      error = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__RadioState__turnOn();

      if (error == SUCCESS) {
        /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__state = /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__STATE_TURN_ON;
        }
    }
  /*CC2420XRadioC.MessageBufferLayerC.MessageBufferLayerP*/MessageBufferLayerP__0__Tasklet__resume();

  return error;
}

# 73 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now)
{
  uint16_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 90
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 147 "/opt/tinyos-2.1.2/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
}

# 96 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
__attribute((wakeup)) __attribute((interrupt(0x0012)))  void sig_UART0RX_VECTOR(void )
#line 96
{
  uint8_t temp = U0RXBUF;

#line 98
  HplMsp430Usart0P__Interrupts__rxDone(temp);
}

# 153 "/opt/tinyos-2.1.2/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void )
#line 153
{
  /* atomic removed: atomic calls only */
#line 154
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 156
        FALSE;

#line 156
        return __nesc_temp;
      }
  }
#line 158
  return TRUE;
}






static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void )
#line 166
{
  /* atomic removed: atomic calls only */
#line 167
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state != /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 169
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;

#line 169
        return __nesc_temp;
      }
#line 170
    {
      unsigned char __nesc_temp = 
#line 170
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId;

#line 170
      return __nesc_temp;
    }
  }
}

# 101 "/opt/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
__attribute((wakeup)) __attribute((interrupt(0x0010)))  void sig_UART0TX_VECTOR(void )
#line 101
{
  if (HplMsp430Usart0P__HplI2C__isI2C()) {
    HplMsp430Usart0P__I2CInterrupts__fired();
    }
  else {
#line 105
    HplMsp430Usart0P__Interrupts__txDone();
    }
}

# 64 "/opt/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0008)))  void sig_PORT1_VECTOR(void )
{
  volatile int n = P1IFG & P1IE;

  if (n & (1 << 0)) {
#line 68
      HplMsp430InterruptP__Port10__fired();
#line 68
      return;
    }
#line 69
  if (n & (1 << 1)) {
#line 69
      HplMsp430InterruptP__Port11__fired();
#line 69
      return;
    }
#line 70
  if (n & (1 << 2)) {
#line 70
      HplMsp430InterruptP__Port12__fired();
#line 70
      return;
    }
#line 71
  if (n & (1 << 3)) {
#line 71
      HplMsp430InterruptP__Port13__fired();
#line 71
      return;
    }
#line 72
  if (n & (1 << 4)) {
#line 72
      HplMsp430InterruptP__Port14__fired();
#line 72
      return;
    }
#line 73
  if (n & (1 << 5)) {
#line 73
      HplMsp430InterruptP__Port15__fired();
#line 73
      return;
    }
#line 74
  if (n & (1 << 6)) {
#line 74
      HplMsp430InterruptP__Port16__fired();
#line 74
      return;
    }
#line 75
  if (n & (1 << 7)) {
#line 75
      HplMsp430InterruptP__Port17__fired();
#line 75
      return;
    }
}

#line 169
__attribute((wakeup)) __attribute((interrupt(0x0002)))  void sig_PORT2_VECTOR(void )
{
  volatile int n = P2IFG & P2IE;

  if (n & (1 << 0)) {
#line 173
      HplMsp430InterruptP__Port20__fired();
#line 173
      return;
    }
#line 174
  if (n & (1 << 1)) {
#line 174
      HplMsp430InterruptP__Port21__fired();
#line 174
      return;
    }
#line 175
  if (n & (1 << 2)) {
#line 175
      HplMsp430InterruptP__Port22__fired();
#line 175
      return;
    }
#line 176
  if (n & (1 << 3)) {
#line 176
      HplMsp430InterruptP__Port23__fired();
#line 176
      return;
    }
#line 177
  if (n & (1 << 4)) {
#line 177
      HplMsp430InterruptP__Port24__fired();
#line 177
      return;
    }
#line 178
  if (n & (1 << 5)) {
#line 178
      HplMsp430InterruptP__Port25__fired();
#line 178
      return;
    }
#line 179
  if (n & (1 << 6)) {
#line 179
      HplMsp430InterruptP__Port26__fired();
#line 179
      return;
    }
#line 180
  if (n & (1 << 7)) {
#line 180
      HplMsp430InterruptP__Port27__fired();
#line 180
      return;
    }
}

