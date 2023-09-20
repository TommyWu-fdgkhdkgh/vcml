/******************************************************************************
 *                                                                            *
 * Copyright (C) 2023 MachineWare GmbH                                        *
 * All Rights Reserved                                                        *
 *                                                                            *
 * This is work is licensed under the terms described in the LICENSE file     *
 * found in the root directory of this source tree.                           *
 *                                                                            *
 ******************************************************************************/

#include "vcml/models/dma/sifive_pdma.h"

namespace vcml {
namespace dma {

void sifive_pdma::sifive_pdma_run(size_t ch) {
    uint32_t cfg = *next_cfg[ch];
    uint64_t bytes = *next_bytes[ch];
    uint64_t dst = *next_dst[ch];
    uint64_t src = *next_src[ch];

    uint64_t n, wsize, rsize, size, remainder;
    uint8_t buf[64];

    /* do nothing if bytes to transfer is zero */
    if (!bytes) {
        goto done;
    }

    /*
     * The manual does not describe how the hardware behaviors when
     * config.wsize and config.rsize are given different values.
     * A common case is memory to memory DMA, and in this case they
     * are normally the same. Abort if this expectation fails.
     */
    wsize = (cfg >> CONFIG_WRSZ_SHIFT) & CONFIG_SZ_MASK;
    rsize = (cfg >> CONFIG_RDSZ_SHIFT) & CONFIG_SZ_MASK;

    if (wsize != rsize) {
        goto error;
    }

    /*
     * Calculate the transaction size
     *
     * size field is base 2 logarithm of DMA transaction size,
     * but there is an upper limit of 64 bytes per transaction.
     */
    size = wsize;
    if (size > 6) {
        size = 6;
    }
    size = 1 << size;
    remainder = bytes % size;

    /* indicate a DMA transfer is started */
    state[ch] = DMA_CHAN_STATE_STARTED;
    *control[ch] &= ~CONTROL_DONE;
    *control[ch] &= ~CONTROL_ERR;

    /* load the next_ registers into their exec_ counterparts */
    *exec_cfg[ch] = cfg;
    *exec_bytes[ch] = bytes;
    *exec_dst[ch] = dst;
    *exec_src[ch] = src;

    for (n = 0; n < bytes / size; n++) {
        out.read(*exec_src[ch], buf, size);
        out.write(*exec_dst[ch], buf, size);

        *exec_src[ch] += size;
        *exec_dst[ch] += size;
        *exec_bytes[ch] -= size;
    }

    if (remainder) {
        out.read(*exec_src[ch], buf, remainder);
        out.write(*exec_dst[ch], buf, remainder);

        *exec_src[ch] += remainder;
        *exec_dst[ch] += remainder;
        *exec_bytes[ch] -= remainder;
    }

    /* reload exec_ registers if repeat is required */
    if (*next_cfg[ch] & CONFIG_REPEAT) {
        *exec_bytes[ch] = bytes;
        *exec_dst[ch] = dst;
        *exec_src[ch] = src;
    }

done:
    /* indicate a DMA transfer is done */
    state[ch] = DMA_CHAN_STATE_DONE;
    *control[ch] &= ~CONTROL_RUN;
    *control[ch] |= (uint32_t)CONTROL_DONE;
    return;

error:
    state[ch] = DMA_CHAN_STATE_ERROR;
    *control[ch] |= (uint32_t)CONTROL_ERR;
    return;
}

void sifive_pdma::sifive_pdma_update_irq(size_t ch) {
    bool done_ie, err_ie;

    done_ie = !!(*control[ch] & CONTROL_DONE_IE);
    err_ie = !!(*control[ch] & CONTROL_ERR_IE);

    if (done_ie && (*control[ch] & CONTROL_DONE)) {
        irqs[ch * 2] = true;
    } else {
        irqs[ch * 2] = false;
    }

    if (err_ie && (*control[ch] & CONTROL_ERR)) {
        irqs[ch * 2 + 1] = true;
    } else {
        irqs[ch * 2 + 1] = false;
    }

    state[ch] = DMA_CHAN_STATE_IDLE;
}

void sifive_pdma::write_control(u32 value, size_t ch) {
    bool claimed = !!(*control[ch] & CONTROL_CLAIM);
    bool run = !!(*control[ch] & CONTROL_RUN);

    if (!claimed && (value & CONTROL_CLAIM)) {
        /* reset next* registers */
        *next_cfg[ch] = (CONFIG_RDSZ_DEFAULT << CONFIG_RDSZ_SHIFT) |
                        (CONFIG_WRSZ_DEFAULT << CONFIG_WRSZ_SHIFT);
        *next_bytes[ch] = 0;
        *next_dst[ch] = 0;
        *next_src[ch] = 0;
    }

    /* claim bit can only be cleared when run is low */
    if (run && !(value & CONTROL_CLAIM)) {
        value |= CONTROL_CLAIM;
    }

    *control[ch] = value;

    /*
     * If channel was not claimed before run bit is set,
     * or if the channel is disclaimed when run was low,
     * DMA won't run.
     */
    if (!claimed || (!run && !(value & CONTROL_CLAIM))) {
        *control[ch] &= ~CONTROL_RUN;
        return;
    }

    if (value & CONTROL_RUN) {
        sifive_pdma_run(ch);
    }

    sifive_pdma_update_irq(ch);
}

sifive_pdma::sifive_pdma(const sc_module_name& nm):
    peripheral(nm),
    in("in"),
    out("out"),
    irqs("irqs", SIFIVE_PDMA_IRQS) {

    for (size_t i = 0; i < SIFIVE_PDMA_CHANS; i++) {
        const string rnm_control = mkstr("control_%zu", i); 
        control[i] = new reg<u32>(rnm_control, 0x1000 * i + 0x0, 0x0); 
        control[i]->allow_read_write();
        control[i]->on_write(&sifive_pdma::write_control);
        control[i]->tag = i;

        const string rnm_next_cfg = mkstr("next_cfg_%zu", i);
        next_cfg[i] = new reg<u32>(rnm_next_cfg, 0x1000 * i + 0x4, 0x0);
        next_cfg[i]->allow_read_write();
        next_cfg[i]->tag = i;

        const string rnm_next_bytes = mkstr("next_bytes_%zu", i);
        next_bytes[i] = new reg<u64>(rnm_next_bytes, 0x1000 * i + 0x8, 0x0);
        next_bytes[i]->allow_read_write();
        next_bytes[i]->tag = i;

        const string rnm_next_dst = mkstr("next_dst_%zu", i);
        next_dst[i] = new reg<u64>(rnm_next_dst, 0x1000 * i + 0x10, 0x0);
        next_dst[i]->allow_read_write();
        next_dst[i]->tag = i;

        const string rnm_next_src = mkstr("next_src_%zu", i);
        next_src[i] = new reg<u64>(rnm_next_src, 0x1000 * i + 0x18, 0x0);
        next_src[i]->allow_read_write();
        next_src[i]->tag = i;

        const string rnm_exec_cfg = mkstr("exec_cfg_%zu", i);
        exec_cfg[i] = new reg<u32>(rnm_exec_cfg, 0x1000 * i + 0x104, 0x0);
        exec_cfg[i]->allow_read_only();
        exec_cfg[i]->tag = i;

        const string rnm_exec_bytes = mkstr("exec_bytes_%zu", i);
        exec_bytes[i] = new reg<u64>(rnm_exec_bytes, 0x1000 * i + 0x108, 0x0);
        exec_bytes[i]->allow_read_only();
        exec_bytes[i]->tag = i;

        const string rnm_exec_dst = mkstr("exec_dst_%zu", i);
        exec_dst[i] = new reg<u64>(rnm_exec_dst, 0x1000 * i + 0x110, 0x0);
        exec_dst[i]->allow_read_only();
        exec_dst[i]->tag = i;

        const string rnm_exec_src = mkstr("exec_src_%zu", i);
        exec_src[i] = new reg<u64>(rnm_exec_src, 0x1000 * i + 0x118, 0x0);
        exec_src[i]->allow_read_only();
        exec_src[i]->tag = i;
    }
}

sifive_pdma::~sifive_pdma() {
    // nothing to do
}

void sifive_pdma::reset() {
    peripheral::reset();
}

VCML_EXPORT_MODEL(vcml::dma::sifive_pdma, name, args) {
    return new sifive_pdma(name);
}

} // namespace dma
} // namespace vcml
