#ifndef dma_H
#define dma_H

#include "mp_bbn2123.h"
#include "system_mp_bbn2123.h"

#define DMA_NO_CH       8

typedef struct DmaMap_ {
    volatile uint32_t LISR;
    volatile uint32_t HISR;
    volatile uint32_t LIFCR;
    volatile uint32_t HIFCR;
    struct {
        volatile uint32_t CR;
        volatile uint32_t NDTR;
        volatile uint32_t PAR;
        volatile uint32_t M0AR;
        volatile uint32_t M1AR;
        volatile uint32_t FCR;
    } STREAM[DMA_NO_CH];
} DmaMap;

#define DMA ((DmaMap*)(BA_DMA))

typedef enum DmaBurst_
{
    burstSingle,
    burstInc4,
    burstInc8,
    burstInc16
} DmaBurst;

typedef enum DmaDbm_
{
    dbmNo,
    dbmYes
} DmaDbm;

typedef enum DmaPriority_
{
    plLow,
    plMedium,
    plHigh,
    plVeryHigh
} DmaPriority;

typedef enum DmaPincos_
{
    pincosFromPsize,
    pincosFix4Bytes
} DmaPincos;

typedef enum DmaSize_
{
    sizeByte,
    sizeHword,
    sizeWord
} DmaSize;

typedef enum DmaIncMode_
{
    incModeFix,
    incModeInc
} DmaIncMode;

typedef enum DmaCirc_
{
    circDis,
    circEn
} DmaCirc;

typedef enum DmaDir_
{
    dirPerToMem,
    dirMemToPer,
    dirMemToMem
} DmaDir;

typedef enum DmaDirMode_
{
    dirModeEn,
    dirModeDis
} DmaDirMode;

typedef enum DmaFifoTh_
{
    fthQuarter,
    fthHalf,
    fthThreeQuarter,
    fthFull
} DmaFifoTh;

typedef struct DmaStream__ {
    volatile uint8_t stream;
    struct {
        volatile uint32_t chsel     : 3;
        volatile DmaBurst mburst    : 2;
        volatile DmaBurst pburst    : 2;
        volatile uint32_t ct        : 1;
        volatile DmaDbm dbm         : 1;
        volatile DmaPriority pl     : 2;
        volatile DmaPincos pincos   : 1;
        volatile DmaSize msize      : 2;
        volatile DmaSize psize      : 2;
        volatile DmaIncMode minc    : 1;
        volatile DmaIncMode pinc    : 1;
        volatile DmaCirc circ       : 1;
        volatile DmaDir dir         : 2;
        volatile uint32_t pfctrl    : 1;
        volatile uint32_t tcie      : 1;
        volatile uint32_t htie      : 1;
        volatile uint32_t teie      : 1;
        volatile uint32_t dmeie     : 1;
    } ctrl;
    volatile uint16_t ndt;
    volatile uint32_t pa;
    volatile uint32_t m0a;
    volatile uint32_t m1a;
    struct {
        volatile uint8_t    feie    : 1;
        volatile DmaDirMode dmdis   : 1;
        volatile DmaFifoTh  fth     : 2;
    } fctrl;
} DmaStream;

void dma_init(DmaStream *dma, int8_t stream, uint32_t pa, uint32_t m0a, DmaDir dir, uint16_t ndt);
void dma_config(DmaStream *dma);
void dma_set_en(DmaStream *dma);
void dma_wait_en(DmaStream *dma, int8_t val);
void dma_set_ctrl_reg(DmaStream *dma);
void dma_set_ndt_reg(DmaStream *dma);
void dma_set_pa_reg(DmaStream *dma);
void dma_set_m0a_reg(DmaStream *dma);
void dma_set_m1a_reg(DmaStream *dma);
void dma_set_fctrl_reg(DmaStream *dma);

#endif
