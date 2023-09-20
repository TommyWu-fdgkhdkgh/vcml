/******************************************************************************
 *                                                                            *
 * Copyright (C) 2023 MachineWare GmbH                                        *
 * All Rights Reserved                                                        *
 *                                                                            *
 * This is work is licensed under the terms described in the LICENSE file     *
 * found in the root directory of this source tree.                           *
 *                                                                            *
 ******************************************************************************/

#ifndef VCML_DMA_SIFIVE_PDMA_H
#define VCML_DMA_SIFIVE_PDMA_H

#include "vcml/core/types.h"
#include "vcml/core/systemc.h"
#include "vcml/core/range.h"
#include "vcml/core/peripheral.h"
#include "vcml/core/model.h"

#include "vcml/protocols/tlm.h"
#include "vcml/protocols/gpio.h"
#include "vcml/protocols/serial.h"

namespace vcml {
namespace dma {

class sifive_pdma : public peripheral
{
private:
    static const u32 SIFIVE_PDMA_CHANS = 4;
    static const u32 SIFIVE_PDMA_IRQS = SIFIVE_PDMA_CHANS * 2;

    static const u32 CONTROL_CLAIM = bit(0);
    static const u32 CONTROL_RUN = bit(1);
    static const u32 CONTROL_DONE_IE = bit(14);
    static const u32 CONTROL_ERR_IE = bit(15);
    static const u32 CONTROL_DONE = bit(30);
    static const u32 CONTROL_ERR = bit(31);

    static const u32 CONFIG_REPEAT = bit(2);
    static const u32 CONFIG_ORDER = bit(3);
    static const u32 CONFIG_WRSZ_SHIFT = 24;
    static const u32 CONFIG_RDSZ_SHIFT = 28;
    static const u32 CONFIG_SZ_MASK = 0xf;
    static const u32 CONFIG_WRSZ_DEFAULT = 6;
    static const u32 CONFIG_RDSZ_DEFAULT = 6;

    enum dma_chan_state {
        DMA_CHAN_STATE_IDLE,
        DMA_CHAN_STATE_STARTED,
        DMA_CHAN_STATE_ERROR,
        DMA_CHAN_STATE_DONE
    };

    void write_control(u32 value, size_t ch);
    void write_next_cfg(u32 value, size_t ch);
    void write_next_bytes(u64 value, size_t ch);
    void write_next_dst(u64 value, size_t ch);
    void write_next_src(u64 value, size_t ch);

    void sifive_pdma_run(size_t ch);
    void sifive_pdma_update_irq(size_t ch);

public:
    reg<u32> *control[SIFIVE_PDMA_CHANS];    // Channel Control Register
    reg<u32> *next_cfg[SIFIVE_PDMA_CHANS];   // Next transfer type
    reg<u64> *next_bytes[SIFIVE_PDMA_CHANS]; // Number of bytes to move
    reg<u64> *next_dst[SIFIVE_PDMA_CHANS];   // Destination start address
    reg<u64> *next_src[SIFIVE_PDMA_CHANS];   // Source start address
    reg<u32> *exec_cfg[SIFIVE_PDMA_CHANS];   // Active transfer type
    reg<u64> *exec_bytes[SIFIVE_PDMA_CHANS]; // Number of bytes remaining
    reg<u64> *exec_dst[SIFIVE_PDMA_CHANS];   // Destination current address
    reg<u64> *exec_src[SIFIVE_PDMA_CHANS];   // Source current address
    u32 state[SIFIVE_PDMA_CHANS];

    tlm_target_socket in;
    tlm_initiator_socket out;

    gpio_initiator_array irqs;

    sifive_pdma(const sc_module_name& name);
    virtual ~sifive_pdma();
    VCML_KIND(dma::sifive_pdma);
    virtual void reset() override;

    sifive_pdma() = delete;
    sifive_pdma(const sifive_pdma&) = delete;
};

} // namespace pdma
} // namespace vcml

#endif
