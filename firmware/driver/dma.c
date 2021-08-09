#include "dma.h"

void dma_init(DmaStream *dma, int8_t stream, uint32_t pa, uint32_t m0a, DmaDir dir, uint16_t ndt) {
    dma->stream = stream;

    dma->ctrl.chsel = 0;
    dma->ctrl.mburst = burstSingle;
    dma->ctrl.pburst = burstSingle;
    dma->ctrl.ct = 0;
    dma->ctrl.dbm = dbmNo;
    dma->ctrl.pl = plLow;
    dma->ctrl.pincos = pincosFromPsize;
    dma->ctrl.msize = sizeByte;
    dma->ctrl.psize = sizeByte;
    dma->ctrl.minc = incModeInc;
    dma->ctrl.pinc = incModeFix;
    dma->ctrl.circ = circDis;
    dma->ctrl.dir = dir;
    dma->ctrl.pfctrl = 0;
    dma->ctrl.tcie = 0;
    dma->ctrl.htie = 0;
    dma->ctrl.teie = 0;
    dma->ctrl.dmeie = 0;
    dma->ndt = ndt;
    dma->pa = pa;
    dma->m0a = m0a;
    dma->m1a = 0;
    dma->fctrl.feie = 0;
    dma->fctrl.dmdis = dirModeDis;
    dma->fctrl.fth = fthFull;
}

void dma_config(DmaStream *dma) {
    dma_set_ctrl_reg(dma);
    dma_set_ndt_reg(dma);
    dma_set_pa_reg(dma);
    dma_set_m0a_reg(dma);
    dma_set_m1a_reg(dma);
    dma_set_fctrl_reg(dma);
    dma_set_en(dma);
}

void dma_set_en(DmaStream *dma) {
    DMA->STREAM[dma->stream].CR = DMA->STREAM[dma->stream].CR | 1;
}

void dma_wait_en(DmaStream *dma, int8_t val) {
    while((DMA->STREAM[dma->stream].CR & 1) != (val & 1));
}

void dma_set_ctrl_reg(DmaStream *dma) {
    uint32_t tmp = dma->ctrl.chsel << 25;
    tmp |= dma->ctrl.mburst << 23;
    tmp |= dma->ctrl.pburst << 21;
    tmp |= dma->ctrl.ct << 19;
    tmp |= dma->ctrl.dbm << 18;
    tmp |= dma->ctrl.pl << 16;
    tmp |= dma->ctrl.pincos << 15;
    tmp |= dma->ctrl.msize << 13;
    tmp |= dma->ctrl.psize << 11;
    tmp |= dma->ctrl.minc << 10;
    tmp |= dma->ctrl.pinc << 9;
    tmp |= dma->ctrl.circ << 8;
    tmp |= dma->ctrl.dir << 6;
    tmp |= dma->ctrl.pfctrl << 5;
    tmp |= dma->ctrl.tcie << 4;
    tmp |= dma->ctrl.htie << 3;
    tmp |= dma->ctrl.teie << 2;
    tmp |= dma->ctrl.dmeie << 1;
    DMA->STREAM[dma->stream].CR = tmp;
}

void dma_set_ndt_reg(DmaStream *dma) {
    DMA->STREAM[dma->stream].NDTR = dma->ndt;
}

void dma_set_pa_reg(DmaStream *dma) {
    DMA->STREAM[dma->stream].PAR = dma->pa;
}

void dma_set_m0a_reg(DmaStream *dma) {
    DMA->STREAM[dma->stream].M0AR = dma->m0a;
}

void dma_set_m1a_reg(DmaStream *dma) {
    DMA->STREAM[dma->stream].M1AR = dma->m1a;
}

void dma_set_fctrl_reg(DmaStream *dma) {
    uint32_t tmp = dma->fctrl.feie << 7;
    tmp |= dma->fctrl.dmdis << 2;
    tmp |= dma->fctrl.fth;
    DMA->STREAM[dma->stream].FCR = tmp;
}
