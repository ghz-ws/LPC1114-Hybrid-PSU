#include "mbed.h"

//BufferedSerial uart0(P1_7, P1_6,115200);  //TX, RX
SPI spi(P0_9, P0_8, P0_6);    //mosi, miso, sclk
I2C i2c(P0_5,P0_4);
DigitalOut ldac(P0_1);
DigitalOut cs1(P0_7);   //ADC
DigitalOut cs2(P0_11);  //DAC
DigitalOut rc(P1_5);
DigitalIn drdy(P0_3);
DigitalIn on(P1_4);
DigitalIn a1(P1_1);
DigitalIn b1(P1_8);
DigitalIn a2(P1_9);
DigitalIn b2(P1_2);

//OLED
const uint8_t oled1=0x78;   //oled i2c addr 0x3c<<1
void oled_init(uint8_t addr);     //lcd init func
void char_disp(uint8_t addr, uint8_t position, char data);    //char disp func
void cont(uint8_t addr,uint8_t val);     //contrast set
void vs_disp(uint8_t addr, uint8_t position, int16_t val);
void vm_disp(uint8_t addr, uint8_t position, float val);
void im_disp(uint8_t addr, uint8_t position, float val);
void tm_disp(uint8_t addr, uint8_t position, uint8_t val);
void off_disp(uint8_t addr);

//Rotary state and display
const uint16_t tc_on=3500;    //lcd tick on time
const uint16_t tc_off=2500;    //lcd tick off time
const uint16_t disp_ref=12000;      //val disp refresh rate
const uint16_t temp_ref=12000;      //val disp refresh rate
uint16_t tc, disp_c, temp_c;          //tick counter, temp disp refresh counter
uint8_t r1_state, r2_state;       //rotary 1 state, rotary 2 state
uint8_t r1_val, r2_val;       //0->idle, 1->incr, 2->decr
uint8_t cur_pos, tick_pos;

//adc/dac control
const uint8_t rst=0b0110;
const uint8_t wreg=0b0100;
const uint8_t start=0b1000;
void drdy_wait();
uint16_t adc_read(uint8_t ch, uint8_t avg);
void dac_send(uint8_t ch, uint16_t val);
void v_set(int16_t val);

//set
#define gv 12.5     //ADA4522 voltage set amp gain
#define dac_res 65536   //2^16 dac resolution
#define dac_ref 2500   //dac FS. mV unit
#define vs_cal 1.000233//1.000267 //vs calib
int16_t vs, vs_p;   //mV unit
uint8_t en_p;

//meas
#define adc_res 32768 //2^15 adc resolution. unipolar
#define adc_ref 2048   //2.048V adc reference
#define att 0.052632   //vm att gain
#define il 5000     //Iout=ILOAD/5000
#define ir 6800    //IMON resistor
#define td 0.000001     //1uA/C
#define tr 10000    //TMON resistor
#define im_cal 0.985616//0.979824 //im calib
#define vm_cal 1.006070//0.998569 //vm calib
float im, vm, tm;   //mV, mA, C unit
uint8_t tm_int; //C unit

int main(){
    spi.format(8,1);
    spi.frequency(5000000); //SPI clk 5MHz
    i2c.frequency(400000);  //I2C clk 400kHz
    cs1=1;
    cs2=1;
    ldac=0;
    rc=0;

    //OLED init
    thread_sleep_for(50);  //wait for OLED power on
    oled_init(oled1);
    thread_sleep_for(10);
    cont(oled1,0xff);
    
    //adc init
    cs1=0;
    spi.write(rst);
    cs1=1;
    thread_sleep_for(10);
    cs1=0;
    spi.write((wreg<<4)+(0x01<<2)+0);    //write addr 0x01, 1byte
    spi.write((0b010<<5)+(0b10<<3)+(0b1<<2));       //180sps, turbo mode, cc mode
    cs1=1;

    char_disp(oled1,7,'V');
    char_disp(oled1,11,'T');
    char_disp(oled1,12,'=');
    char_disp(oled1,15,'C');
    char_disp(oled1,0x20+7,'V');
    char_disp(oled1,0x20+15,'A');
    off_disp(oled1);
    im_disp(oled1,0x20+9,0);

    while (true){
        //rotary scan
        r1_state=((r1_state<<1)+a1)&0b0011;
        if((r1_state==1)&&(b1==0))r1_val=1;      //r1 incr
        else if((r1_state==1)&&(b1==1))r1_val=2; //r1 decr

        r2_state=((r2_state<<1)+a2)&0b0011;
        if((r2_state==1)&&(b2==0))r2_val=1;      //r2 incr
        else if((r2_state==1)&&(b2==1))r2_val=2; //r2 decr        

        if(r2_val==1){
            if(cur_pos<5) ++cur_pos;
            else cur_pos=0;
        }else if(r2_val==2){
            if(cur_pos>0) --cur_pos;
            else cur_pos=5;
        }

        if(cur_pos==0){
            tick_pos=16;
        }else if(cur_pos==1){
            tick_pos=0;
            if(r1_val==1)vs=vs+10000;
            else if(r1_val==2) vs=vs-10000;
        }else if(cur_pos==2){
            tick_pos=1;
            if(r1_val==1)vs=vs+1000;
            else if(r1_val==2) vs=vs-1000;
        }else if(cur_pos==3){
            tick_pos=3;
            if(r1_val==1)vs=vs+100;
            else if(r1_val==2) vs=vs-100;
        }else if(cur_pos==4){
            tick_pos=4;
            if(r1_val==1)vs=vs+10;
            else if(r1_val==2) vs=vs-10;
        }else if(cur_pos==5){
            tick_pos=5;
            if(r1_val==1)vs=vs+1;
            else if(r1_val==2) vs=vs-1;
        }

        r1_val=0;
        r2_val=0;

        //vset overflow check
        if(vs>=30000)vs=30000;
        if(vs<=0)vs=0;

        //vset display
        if(tc==0){
            vs_disp(oled1,0,vs);
            tc++;
        }else if((tc>0)&&(tc<tc_on)){
            if(vs_p!=vs){
                vs_disp(oled1,0,vs);
            }
            tc++; 
        }else if(tc==tc_on){
            char_disp(oled1,tick_pos,' ');
            tc++;
        }else if((tc>tc_on)&&(tc<(tc_on+tc_off))){
            tc++;
        }else{
            tc=0;
        }

        if((en_p==1)&&(on==1)){  //sw off changed
            v_set(0);
            rc=0;
            off_disp(oled1);
            im_disp(oled1,0x20+9,0);
            en_p=0;
        }else if((en_p==0)&&(on==1)){  //sw off steady
            //nothing
        }else if((en_p==0)&&(on==0)){  //sw on changed
            v_set(vs);
            rc=1;
            en_p=1;
        }else{      //sw on steady
            if(vs_p!=vs)v_set(vs);;
            if(disp_c==disp_ref){
                im=(float)adc_read(2,2)*adc_ref/adc_res*il/ir*im_cal; //IM, mA unit
                im_disp(oled1,0x20+9,im);
                disp_c=0;
            }else if(disp_c==(disp_ref/2)){
                vm=(float)adc_read(0,2)*adc_ref/adc_res/att*vm_cal;   //VM, mV unit
                vm_disp(oled1,0x20+0,vm);
                disp_c++;
            }else{
                disp_c++;
            }
        }
        vs_p=vs;
        if(temp_c==temp_ref){
            tm=(float)adc_read(1,3)*adc_ref/adc_res/tr/td/1000;   //TM, C unit
            tm_int=(uint8_t)tm;
            tm_disp(oled1,13,tm_int);
            temp_c=0;
        }else{
            temp_c++;
        }
    }
}

//LCD func
void oled_init(uint8_t addr){
    char lcd_data[2];
    lcd_data[0] = 0x0;
    lcd_data[1]=0x01;           //0x01 clear disp
    i2c.write(addr, lcd_data, 2);
    thread_sleep_for(20);
    lcd_data[1]=0x02;           //0x02 return home
    i2c.write(addr, lcd_data, 2);
    thread_sleep_for(20);
    lcd_data[1]=0x0C;           //0x0c disp on
    i2c.write(addr, lcd_data, 2);
    thread_sleep_for(20);
    lcd_data[1]=0x01;           //0x01 clear disp
    i2c.write(addr, lcd_data, 2);
    thread_sleep_for(20);
}

void char_disp(uint8_t addr, uint8_t position, char data){
    char buf[2];
    buf[0]=0x0;
    buf[1]=0x80+position;   //set cusor position (0x80 means cursor set cmd)
    i2c.write(addr,buf, 2);
    buf[0]=0x40;            //ahr disp cmd
    buf[1]=data;
    i2c.write(addr,buf, 2);
}

void cont(uint8_t addr,uint8_t val){
    char buf[2];
    buf[0]=0x0;
    buf[1]=0x2a;
    i2c.write(addr,buf,2);
    buf[1]=0x79;    //SD=1
    i2c.write(addr,buf,2);
    buf[1]=0x81;    //contrast set
    i2c.write(addr,buf,2);
    buf[1]=val;    //contrast value
    i2c.write(addr,buf,2);
    buf[1]=0x78;    //SD=0
    i2c.write(addr,buf,2);
    buf[1]=0x28;    //0x2C, 0x28
    i2c.write(addr,buf,2);
}

//dac func
void dac_send(uint8_t ch, uint16_t val){
    uint8_t cmd=0b0011; //Write & update
    uint8_t addr;
    cs2=0;
    if(ch==0) addr=0b0001;  //DAC A
    else addr=0b1000;       //DAC B
    spi.write((cmd<<4)+addr);  //cmd & addr
    spi.write(val>>8);          //MSB
    spi.write(val&0xff);        //LSB
    cs2=1;
}

//adc func
void drdy_wait(){
    while(true){
        if(drdy==0) break;
    }
}

uint16_t adc_read(uint8_t ch, uint8_t avg){
    uint8_t i=0;
    uint8_t buf[2];     //spi receive buf
    int16_t raw_val;
    int32_t total;
    cs1=0;
    spi.write((wreg<<4));       //write addr 0x00, 1byte
    if(ch==0)spi.write((0b1000<<4)+1);   //ch0 mux, pga disable
    if(ch==1)spi.write((0b1001<<4)+1);   //ch1 mux, pga disable
    if(ch==2)spi.write((0b1010<<4)+1);   //ch2 mux, pga disable
    if(ch==3)spi.write((0b1011<<4)+1);   //ch3 mux, pga disable
    cs1=1;
    for (i=0;i<avg;++i){
        drdy_wait();
        cs1=0;
        buf[1]=spi.write(0x00);
        buf[0]=spi.write(0x00);
        cs1=1;
        raw_val=(buf[1]<<8)+buf[0];
        total=total+(int32_t)raw_val;
    }
    if(total<0)total=0;
    return (uint16_t)(total/avg);
}

void vs_disp(uint8_t addr, uint8_t position, int16_t val){
    char buf[7];
    buf[0]=0x0;
    buf[1]=0x80+position;   //set cusor position (0x80 means cursor set cmd)
    i2c.write(addr,buf,2);
    buf[0]=0x40;        //write cmd
    buf[1]=0x30+(val/10000)%10;//10
    buf[2]=0x30+(val/1000)%10; //1
    buf[3]=0x2E;               //.
    buf[4]=0x30+(val/100)%10;  //0.1
    buf[5]=0x30+(val/10)%10;   //0.01
    buf[6]=0x30+val%10;        //0.001
    i2c.write(addr,buf,7);
}

void vm_disp(uint8_t addr, uint8_t position, float val){
    char buf[7];
    int16_t val_int;
    buf[0]=0x0;
    buf[1]=0x80+position;   //set cusor position (0x80 means cursor set cmd)
    i2c.write(addr,buf,2);
    val_int=(int16_t)val;
    buf[0]=0x40;        //write cmd
    buf[1]=0x30+(val_int/10000)%10;//10
    buf[2]=0x30+(val_int/1000)%10; //1
    buf[3]=0x2E;                   //.
    buf[4]=0x30+(val_int/100)%10;  //0.1
    buf[5]=0x30+(val_int/10)%10;   //0.01
    buf[6]=0x30+val_int%10;        //0.001
    i2c.write(addr,buf,7);
}

void im_disp(uint8_t addr, uint8_t position, float val){
    char buf[6];
    int16_t val_int;
    buf[0]=0x0;
    buf[1]=0x80+position;   //set cusor position (0x80 means cursor set cmd)
    i2c.write(addr,buf,2);
    val_int=(int16_t)val;
    buf[0]=0x40;        //write cmd
    buf[1]=0x30+(val_int/1000)%10; //1
    buf[2]=0x2E;                   //.
    buf[3]=0x30+(val_int/100)%10;  //0.1
    buf[4]=0x30+(val_int/10)%10;   //0.01
    buf[5]=0x30+val_int%10;        //0.001
    i2c.write(addr,buf,6);
}

void tm_disp(uint8_t addr, uint8_t position, uint8_t val){
    char buf[3];
    buf[0]=0x0;
    buf[1]=0x80+position;   //set cusor position (0x80 means cursor set cmd)
    i2c.write(addr,buf, 2);
    if(val>100)val=99;
    buf[0]=0x40;
    buf[1]=0x30+(val/10)%10;   //10
    buf[2]=0x30+val%10;        //1
    i2c.write(addr,buf,3);
}

void off_disp(uint8_t addr){
    char buf[7];
    buf[0]=0x0;
    buf[1]=0x80+0x20+0;   //set cusor position (0x80 means cursor set cmd)
    i2c.write(addr,buf, 2);
    buf[0]=0x40;
    buf[1]=0x20;    //" "
    buf[2]=0x20;    //" "
    buf[3]=0x4F;    //"O"
    buf[4]=0x46;    //"F"
    buf[5]=0x46;    //"F"
    buf[6]=0x20;    //" "
    i2c.write(addr,buf,7);
}

void v_set(int16_t val){
    float vs_f;
    uint16_t send;
    vs_f=(float)vs*dac_res/dac_ref/gv*vs_cal;
    send=(uint16_t)vs_f;
    dac_send(1,send);
}
