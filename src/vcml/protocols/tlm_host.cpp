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

#include "vcml/protocols/tlm_host.h"
#include "vcml/protocols/tlm_sockets.h"

namespace vcml {

    void tlm_host::register_socket(tlm_initiator_socket* socket) {
        if (stl_contains(m_initiator_sockets, socket))
            VCML_ERROR("socket '%s' already registered", socket->name());
        m_initiator_sockets.push_back(socket);
    }

    void tlm_host::register_socket(tlm_target_socket* socket) {
        if (stl_contains(m_target_sockets, socket))
            VCML_ERROR("socket '%s' already registered", socket->name());
        m_target_sockets.push_back(socket);
    }

    void tlm_host::unregister_socket(tlm_initiator_socket* socket) {
        stl_remove_erase(m_initiator_sockets, socket);
    }

    void tlm_host::unregister_socket(tlm_target_socket* socket) {
        stl_remove_erase(m_target_sockets, socket);
    }

    tlm_initiator_socket*
    tlm_host::find_tlm_initiator_socket(const string& name) const {
        for (auto socket : m_initiator_sockets)
            if (name == socket->name())
                return socket;
        return nullptr;
    }

    tlm_target_socket*
    tlm_host::find_tlm_target_socket(const string& name) const {
        for (auto socket : m_target_sockets)
            if (name == socket->name())
                return socket;
        return nullptr;
    }

    vector<tlm_target_socket*>
    tlm_host::find_tlm_target_sockets(address_space as) const {
        vector<tlm_target_socket*> sockets;
        for (auto socket : m_target_sockets)
            if (as == socket->as)
                sockets.push_back(socket);
        return sockets;
    }

    tlm_host::tlm_host(bool allow_dmi):
        m_offsets(),
        m_initiator_sockets(),
        m_target_sockets(),
        allow_dmi("allow_dmi", true) {
    }

    sc_time& tlm_host::local_time(sc_process_b* proc) {
        if (proc == nullptr)
            proc = current_process();

        if (!stl_contains(m_offsets, proc))
             m_offsets[proc] = SC_ZERO_TIME;

        sc_time& local = m_offsets[proc];
        update_local_time(local);
        return local;
    }

    sc_time tlm_host::local_time_stamp(sc_process_b* proc) {
        return sc_time_stamp() + local_time(proc);
    }

    bool tlm_host::needs_sync(sc_process_b* proc) {
        if (proc == nullptr)
            proc = current_process();
        if (!is_thread(proc))
            return false;

        sc_time quantum = tlm::tlm_global_quantum::instance().get();
        return local_time(proc) >= quantum;
    }

    void tlm_host::sync(sc_process_b* proc) {
        if (proc == nullptr)
            proc = current_process();
        if (proc == nullptr || proc->proc_kind() != sc_core::SC_THREAD_PROC_)
            VCML_ERROR("attempt to sync outside of SC_THREAD process");

        sc_time& offset = local_time(proc);
        sc_core::wait(offset);
        offset = SC_ZERO_TIME;
    }

    void tlm_host::map_dmi(const tlm_dmi& dmi) {
        for (auto socket : m_target_sockets)
            socket->map_dmi(dmi);
    }

    void tlm_host::map_dmi(unsigned char* p, u64 start, u64 end, vcml_access a,
        const sc_time& read_latency, const sc_time& write_latency) {
        tlm_dmi dmi;
        dmi.set_dmi_ptr(p);
        dmi.set_start_address(start);
        dmi.set_end_address(end);
        dmi.set_read_latency(read_latency);
        dmi.set_write_latency(write_latency);
        dmi_set_access(dmi, a);
        map_dmi(dmi);
    }

    void tlm_host::unmap_dmi(u64 start, u64 end) {
        for (auto socket : m_target_sockets)
            socket->unmap_dmi(start, end);
    }

    void tlm_host::remap_dmi(const sc_time& rdlat, const sc_time& wrlat) {
        for (auto socket : m_target_sockets)
            socket->remap_dmi(rdlat, wrlat);
    }

    void tlm_host::invalidate_dmi(u64 start, u64 end) {
        // to be overloaded
    }

    void tlm_host::update_local_time(sc_time& local_time) {
        // to be overloaded
    }

    void tlm_host::b_transport(tlm_target_socket& socket,
        tlm_generic_payload& tx, sc_time& dt) {
        sc_process_b* proc = current_thread();
        VCML_ERROR_ON(!proc, "b_transport outside SC_THREAD");
        m_offsets[proc] = dt;
        transport(socket, tx, tx_get_sbi(tx));
        dt = m_offsets[proc];
    }

    unsigned int tlm_host::transport_dbg(tlm_target_socket& socket,
        tlm_generic_payload& tx) {
        sc_time t1 = sc_time_stamp();
        unsigned int bytes = transport(socket, tx, tx_get_sbi(tx) | SBI_DEBUG);
        sc_time t2 = sc_time_stamp();
        if (thctl_is_sysc_thread() && t1 != t2)
            VCML_ERROR("time advance during debug call");
        return bytes;
    }

    bool tlm_host::get_direct_mem_ptr(tlm_target_socket& socket,
        const tlm_generic_payload& tx, tlm_dmi& dmi) {
        return true;
    }

    void tlm_host::invalidate_direct_mem_ptr(tlm_initiator_socket& socket,
        u64 start, u64 end) {
        invalidate_dmi(start, end);
    }

    unsigned int tlm_host::transport(tlm_target_socket& socket,
        tlm_generic_payload& tx, const tlm_sbi& info) {
        return 0; // to be overloaded
    }

}