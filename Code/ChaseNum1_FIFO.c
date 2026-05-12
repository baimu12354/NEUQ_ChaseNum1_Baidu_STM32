#include "ChaseNum1_Include.h"

/***************************FIFO驱动***************************/
void CN_FIFO_init(CN_FIFO_struct *FIFO,uint8_t *pdata,uint16_t templen){
    FIFO->temp=pdata;
    FIFO->head=0;
    FIFO->tail=0;
    FIFO->temp_len=templen;
    FIFO->is_FIFO_full=0;
}

uint8_t CN_FIFO_read(CN_FIFO_struct *FIFO,uint8_t *pdata,uint16_t readlen){
    if(CN_get_FIFO_occupied(FIFO)<readlen){
        return 0;
    }
    else{
        for(int i=0;i<readlen;i++){
            uint8_t byte=0;
            CN_FIFO_read_Byte(FIFO,&byte);
            pdata[i]=byte;
        }
        return 1;
    }
}

uint8_t CN_FIFO_write(CN_FIFO_struct *FIFO,uint8_t *pdata,uint16_t writelen){
    if(CN_get_FIFO_space(FIFO)<writelen){
        return 0;
    }
    else{
        for(int i=0;i<writelen;i++){
            CN_FIFO_write_Byte(FIFO,pdata[i]);
        }
        return 1;
    }
}

uint8_t CN_FIFO_read_Byte(CN_FIFO_struct *FIFO,uint8_t *pdata){//从FIFO尾部读出一个字节，如果失败则返回0
    if(CN_get_FIFO_occupied(FIFO)<1){
        return 0;
    }
    else{
        *pdata=FIFO->temp[FIFO->tail];//从尾部读出一个字节数据
        FIFO->tail++;//尾指针位移
        if(FIFO->tail>=FIFO->temp_len){
            FIFO->tail=0;
        }
        FIFO->is_FIFO_full=0;//清除满缓存状态
        return 1;
    }
}

uint8_t CN_FIFO_write_Byte(CN_FIFO_struct *FIFO,uint8_t data){//从FIFO头部写入一个字节，如果失败则返回0
    if(CN_get_FIFO_space(FIFO)<1){//检查剩余空间
        return 0;
    }
    else{
        FIFO->temp[FIFO->head]=data;//写入数据
        FIFO->head++;//头指针位移
        if(FIFO->head>=FIFO->temp_len){
            FIFO->head=0;
        }
        if(FIFO->head==FIFO->tail){//检查满缓存状态
            FIFO->is_FIFO_full=1;
        }
        return 1;
    }
}

uint16_t CN_get_FIFO_space(CN_FIFO_struct *FIFO){
    return (FIFO->temp_len-CN_get_FIFO_occupied(FIFO));
}

uint16_t CN_get_FIFO_occupied(CN_FIFO_struct *FIFO){
    if(FIFO->head==FIFO->tail){
        if(FIFO->is_FIFO_full){
            return FIFO->temp_len;
        }
        else{
            return 0;
        }
    }
    else{
        if(FIFO->head>FIFO->tail){
            return (FIFO->head-FIFO->tail);
        }
        else{
            return (FIFO->temp_len-(FIFO->tail-FIFO->head));
        }        
    }
}
/***************************FIFO驱动***************************/
