/******************************************************************************
 *                                                                            *
 * Copyright 2021 Jan Henrik Weinstock                                        *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License");            *
 * you may not use this file except in compliance with the License.           *
 * You may obtain a copy of the License at                                    *
 *                                                                            *
 *     http://www.apache.org/licenses/LICENSE-2.0                             *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 *                                                                            *
 ******************************************************************************/

#include "vcml/models/generic/pci_device.h"

namespace vcml { namespace generic {

    const char* pci_cap_str(pci_cap_id id) {
        switch (id) {
        case PCI_CAPABILITY_PM: return "PCI_CAP_PM";
        case PCI_CAPABILITY_MSI: return "PCI_CAP_MSI";
        case PCI_CAPABILITY_MSIX: return "PCI_CAP_MSIX";
        default:
            VCML_ERROR("unknown capability id %d", (int)id);
        }
    }

    pci_capability::pci_capability(pci_device* dev, pci_cap_id cap_id):
        CAP_ID(),
        CAP_NEXT() {
        u8 prev_ptr = dev->pci_cap_ptr;
        dev->pci_cap_ptr = dev->pci_cap_off;

        string name_id = mkstr("%s_ID", pci_cap_str(cap_id));
        string name_next = mkstr("%s_NEXT", pci_cap_str(cap_id));

        CAP_ID = dev->new_cap_reg_ro<u8>(name_id, cap_id);
        CAP_NEXT = dev->new_cap_reg_ro<u8>(name_next, prev_ptr);
    }

    pci_capability::~pci_capability() {
        if (CAP_ID)
            delete CAP_ID;
        if (CAP_NEXT)
            delete CAP_NEXT;
    }

    void pci_cap_msi::set_pending(unsigned int vector, bool set) {
        if (!MSI_PENDING)
            return;

        const u32 mask = 1u << vector;

        if (set)
            *MSI_PENDING |= mask;
        else
            *MSI_PENDING &= ~mask;
    }

    pci_cap_msi::pci_cap_msi(pci_device* dev, u16 control):
        pci_capability(dev, PCI_CAPABILITY_MSI),
        MSI_CONTROL(),
        MSI_ADDR(),
        MSI_ADDR_HI(),
        MSI_DATA(),
        MSI_MASK(),
        MSI_PENDING() {
        MSI_CONTROL = dev->new_cap_reg_rw<u16>("PCI_CAP_MSI_CONTROL", control);
        MSI_CONTROL->write = &pci_device::write_MSI_CONTROL;
        MSI_ADDR = dev->new_cap_reg_rw<u32>("PCI_CAP_MSI_ADDR", 0);
        MSI_ADDR->write = &pci_device::write_MSI_ADDR;

        if (control & PCI_MSI_64BIT)
            MSI_ADDR_HI = dev->new_cap_reg_rw<u32>("PCI_CAP_MSI_ADDR_HI", 0);

        MSI_DATA = dev->new_cap_reg_rw<u16>("PCI_CAP_MSI_DATA", 0);
        dev->pci_cap_off += 2; // reserved space

        if (control & PCI_MSI_VECTOR) {
            MSI_MASK = dev->new_cap_reg_rw<u32>("PCI_CAP_MSI_MASK", 0);
            MSI_MASK->write = &pci_device::write_MSI_MASK;
            MSI_PENDING = dev->new_cap_reg_ro<u32>("PCI_CAP_MSI_PENDING", 0);
        }
    }

    pci_cap_msi::~pci_cap_msi() {
        if (MSI_CONTROL)
            delete MSI_CONTROL;
        if (MSI_ADDR)
            delete MSI_ADDR;
        if (MSI_ADDR_HI)
            delete MSI_ADDR_HI;
        if (MSI_DATA)
            delete MSI_DATA;
        if (MSI_MASK)
            delete MSI_MASK;
        if (MSI_PENDING)
            delete MSI_PENDING;
    }

    pci_device::pci_device(const sc_module_name& nm, const pci_config& cfg):
        peripheral(nm),
        pci_target(),
        pcie("pcie", cfg.pcie),
        PCI_VENDOR_ID(PCI_AS_CFG, "PCI_VENDOR_ID", 0x0, cfg.vendor_id),
        PCI_DEVICE_ID(PCI_AS_CFG, "PCI_DEVICE_ID", 0x2, cfg.device_id),
        PCI_COMMAND(PCI_AS_CFG, "PCI_COMMAND", 0x4, 0),
        PCI_STATUS(PCI_AS_CFG, "PCI_STATUS", 0x6, PCI_STATUS_INIT(cfg.pcie)),
        PCI_REV_ID(PCI_AS_CFG, "PCI_REV_ID", 0x8, cfg.revision_id),
        PCI_HEADER_TYPE(PCI_AS_CFG, "PCI_HEADER_TYPE", 0xe, 0),
        PCI_BAR(PCI_AS_CFG, "PCI_BAR", 0x10, PCI_BAR_UNMAPPED),
        PCI_SUBVENDOR_ID(PCI_AS_CFG,"PCI_SUBVENDOR_ID",0x2c, cfg.subvendor_id),
        PCI_SUBDEVICE_ID(PCI_AS_CFG,"PCI_SUBDEVICE_ID",0x2e, cfg.subsystem_id),
        PCI_CAP_PTR(PCI_AS_CFG, "PCI_CAP_PTR", 0x34, 0),
        PCI_INT_LINE(PCI_AS_CFG, "PCI_INT_LINE", 0x3c, 0),
        PCI_INT_PIN(PCI_AS_CFG, "PCI_INT_PIN", 0x3d, cfg.int_pin),
        PCI_MIN_GRANT(PCI_AS_CFG, "PCI_MIN_GRANT", 0x3e, cfg.min_grant),
        PCI_MAX_LATENCY(PCI_AS_CFG, "PCI_MAX_LATENCY", 0x3f, cfg.max_latency),
        PCIE_XCAP(PCI_AS_CFG, "PCIE_XCAP", 0x100, 0),
        pci_cap_ptr(0),
        pci_cap_off(64),
        m_bars(),
        m_irq(PCI_IRQ_NONE),
        m_msi(nullptr),
        m_msi_notify("msi_notify") {

        PCI_VENDOR_ID.allow_read_only();
        PCI_VENDOR_ID.sync_never();

        PCI_DEVICE_ID.allow_read_only();
        PCI_DEVICE_ID.sync_never();

        PCI_COMMAND.allow_read_write();
        PCI_COMMAND.sync_always();
        PCI_COMMAND.write = &pci_device::write_COMMAND;

        PCI_STATUS.allow_read_write();
        PCI_STATUS.sync_always();
        PCI_STATUS.write = &pci_device::write_STATUS;

        PCI_REV_ID.allow_read_only();
        PCI_REV_ID.sync_never();

        PCI_HEADER_TYPE.allow_read_only();
        PCI_HEADER_TYPE.sync_never();

        PCI_BAR.allow_read_write();
        PCI_BAR.sync_always();
        PCI_BAR.tagged_write = &pci_device::write_BAR;

        PCI_SUBVENDOR_ID.allow_read_only();
        PCI_SUBVENDOR_ID.sync_never();

        PCI_SUBDEVICE_ID.allow_read_only();
        PCI_SUBDEVICE_ID.sync_never();

        PCI_CAP_PTR.allow_read_only();
        PCI_CAP_PTR.sync_never();
        PCI_CAP_PTR.read = &pci_device::read_CAP_PTR;

        PCI_INT_LINE.allow_read_write();
        PCI_INT_LINE.sync_always();

        PCI_INT_PIN.allow_read_only();
        PCI_INT_PIN.sync_always();

        PCIE_XCAP.allow_read_only();
        PCIE_XCAP.sync_never();

        SC_HAS_PROCESS(pci_device);

        if (cfg.msi) {
            m_msi = new pci_cap_msi(this, cfg.msi_control);
            SC_THREAD(msi_process);
        }

        for (u32 i = 0; i < PCI_NUM_BARS; i++) {
            m_bars[i].barno = i;
            m_bars[i].size = 0;
        }
    }

    pci_device::~pci_device() {
        if (m_msi)
            delete m_msi;
    }

    void pci_device::reset() {
        peripheral::reset();

        for (auto& bar : m_bars) {
            bar.addr_lo = PCI_BAR_UNMAPPED & ~(bar.size - 1);
            bar.addr_hi = bar.is_64bit ? PCI_BAR_UNMAPPED : 0u;
        }

        update_bars();
    }

    void pci_device::declare_bar(int barno, u64 size, u32 type) {
        bool is_io = type & PCI_BAR_IO;
        bool is_64 = type & PCI_BAR_64;
        bool is_prefetch = type & PCI_BAR_PREFETCH;
        int maxbar = is_64 ? PCI_NUM_BARS - 1 : PCI_NUM_BARS;

        VCML_ERROR_ON(is_io && is_64, "IO BAR cannot be 64 bit");
        VCML_ERROR_ON(is_io && is_prefetch, "cannot prefetch IO BAR");
        VCML_ERROR_ON(barno >= maxbar, "barno %d out of bounds", barno);

        m_bars[barno].size = size;
        m_bars[barno].is_io = is_io;
        m_bars[barno].is_64bit = is_64;
        m_bars[barno].is_prefetch = is_prefetch;
        m_bars[barno].addr_lo = PCI_BAR_UNMAPPED & ~(size - 1);
        m_bars[barno].addr_hi = is_64 ? PCI_BAR_UNMAPPED : 0u;
    }

    void pci_device::interrupt(bool state, unsigned int vector) {
        if (msi_enabled()) {
            msi_interrupt(state, vector);
        } else {
            legacy_interrupt(state);
        }
    }

    void pci_device::msi_interrupt(bool state, unsigned int vector) {
        VCML_ERROR_ON(!m_msi, "not capable of sending MSIs");
        if (!msi_enabled() || !(PCI_COMMAND & PCI_COMMAND_MMIO))
            return;

        m_msi->set_pending(vector, state);

        if (!m_msi->is_masked(vector) && m_msi->is_pending(vector))
            m_msi_notify.notify(SC_ZERO_TIME);
    }

    void pci_device::legacy_interrupt(bool state) {
        if (state)
            PCI_STATUS |= PCI_STATUS_IRQ;
        else
            PCI_STATUS &= ~PCI_STATUS_IRQ;

        update_irqs();
    }

    void pci_device::pci_transport(pci_target_socket& sck, pci_payload& pci) {
        tlm_generic_payload tx;
        tlm_command cmd = pci_translate_command(pci.command);
        tx_setup(tx, cmd, pci.addr, &pci.data, pci.size);
        peripheral::receive(tx, pci.debug ? SBI_DEBUG : SBI_NONE, pci.space);
        pci.response = pci_translate_response(tx.get_response_status());
    }

    void pci_device::msi_send(unsigned int vector) {
        u32 vmask = m_msi->num_vectors() - 1;
        u32 msi_data = (*m_msi->MSI_DATA & ~vmask) | (vector & vmask);
        u64 msi_addr = *m_msi->MSI_ADDR;

        if (m_msi->is_64bit())
            msi_addr |= (u64)(u32)*m_msi->MSI_ADDR_HI << 32;

        if (!pci_dma_write(msi_addr, sizeof(msi_data), &msi_data))
            log_warn("DMA error while sending MSI%u", vector);
    }

    void pci_device::msi_process() {
        while (true) {
            wait(m_msi_notify);
            for (unsigned int vec = 0; vec < m_msi->num_vectors(); vec++) {
                if (m_msi->is_pending(vec) && !m_msi->is_masked(vec)) {
                    m_msi->set_pending(vec, false);
                    msi_send(vec);
                }
            }
        }
    }

    u32 pci_device::write_BAR(u32 val, u32 barno) {
        PCI_BAR[barno] = val;
        update_bars();
        return PCI_BAR[barno];
    }

    u16 pci_device::write_COMMAND(u16 val) {
        u16 mask = PCI_COMMAND_IO          |
                   PCI_COMMAND_MMIO        |
                   PCI_COMMAND_BUS_MASTER  |
                   PCI_COMMAND_PARITY      |
                   PCI_COMMAND_SERR        |
                   PCI_COMMAND_NO_IRQ;

        if (!pcie) {
            mask |= PCI_COMMAND_SPECIAL    |
                    PCI_COMMAND_INVALIDATE |
                    PCI_COMMAND_PALETTE    |
                    PCI_COMMAND_WAIT       |
                    PCI_COMMAND_FAST_B2B;
        }

        PCI_COMMAND = val & mask;
        update_irqs();
        update_bars();
        return PCI_COMMAND;
    }

    u16 pci_device::write_STATUS(u16 val) {
        u16 mask = PCI_STATUS_MASTER_PARITY_ERROR |
                   PCI_STATUS_TX_TARGET_ABORT |
                   PCI_STATUS_RX_TARGET_ABORT |
                   PCI_STATUS_RX_MASTER_ABORT |
                   PCI_STATUS_TX_SYSTEM_ERROR |
                   PCI_STATUS_PARITY_ERROR;
        return PCI_STATUS & ~(val & mask);
    }

    u16 pci_device::write_MSI_CONTROL(u16 val) {
        size_t num_vectors = (val & PCI_MSI_QSIZE) >> 4;
        if (num_vectors > m_msi->max_vectors()) {
            log_warn("exceeding max MSI vectors %zu", num_vectors);
            num_vectors = m_msi->max_vectors();
            val = (val & ~PCI_MSI_QSIZE) | num_vectors << 4;
        }

        u16 mask = PCI_MSI_ENABLE | PCI_MSI_QSIZE;
        return (*m_msi->MSI_CONTROL & ~mask) | (val & mask);
    }

    u32 pci_device::write_MSI_ADDR(u32 val) {
        return val & ~0x3;
    }

    u32 pci_device::write_MSI_MASK(u32 val) {
        *m_msi->MSI_MASK = val & ((1ul << m_msi->num_vectors()) - 1);
        m_msi_notify.notify(SC_ZERO_TIME);
        return *m_msi->MSI_MASK;
    }

    void pci_device::update_bars() {
        for (int barno = 0; barno < PCI_NUM_BARS; barno++) {
            pci_bar* bar = m_bars + barno;
            if (bar->size == 0)
                continue;

            PCI_BAR[barno] &= bar->mask();

            if (!(PCI_COMMAND & PCI_COMMAND_IO) && bar->is_io)
                PCI_BAR[barno] = PCI_BAR_UNMAPPED;
            if (!(PCI_COMMAND & PCI_COMMAND_MMIO) && !bar->is_io)
                PCI_BAR[barno] = PCI_BAR_UNMAPPED;
            if (bar->is_io)
                PCI_BAR[barno] |= PCI_BAR_IO;
            if (bar->is_64bit)
                PCI_BAR[barno] |= PCI_BAR_64;
            if (bar->is_prefetch)
                PCI_BAR[barno] |= PCI_BAR_PREFETCH;

            u64 addr = PCI_BAR[barno] & bar->mask();
            if (bar->is_64bit)
                addr |= (u64)PCI_BAR[barno + 1] << 32;

            u32 lo = bar->addr_lo;
            bar->addr = addr;

            if (lo == bar->addr_lo)
                continue;

            if (bar->addr_lo == (PCI_BAR_UNMAPPED & bar->mask())) {
                log_debug("unmapping BAR%d", barno);
                pci_bar_unmap(barno);
            } else {
                log_debug("mapping BAR%d to %s 0x%lx..0x%lx", barno,
                          bar->is_io ? "IO" : "MMIO", bar->addr,
                          bar->addr + bar->size -1);
                pci_bar_map(m_bars[barno]);
            }
        }
    }

    void pci_device::update_irqs() {
        pci_irq irq = PCI_IRQ_NONE;
        if ((PCI_STATUS & PCI_STATUS_IRQ) && !(PCI_COMMAND & PCI_COMMAND_NO_IRQ))
            irq = (pci_irq)(u8)PCI_INT_PIN;

        if (irq == m_irq)
            return;

        if (m_irq != PCI_IRQ_NONE)
            pci_interrupt(m_irq, false);

        m_irq = irq;

        if (m_irq != PCI_IRQ_NONE)
            pci_interrupt(m_irq, true);
    }

}}