# Copyright (c) 2026 Daniel J. Mazure
# SPDX-License-Identifier: BSD-3-Clause
#
# powerlib - device tree for OpenPiton + Microwatt (OpenPOWER), the sibling of
# riscvlib.gen_riscv_dts: the same devices*.xml that pyhplib turns into the
# chipset's address decode (packet_filter, chipset_impl) is turned here into
# the DTS that Linux on the Microwatt tiles boots with, so the two cannot
# disagree by construction.
#
# The one translation between them is the Microwatt tile's core address map
# (rr-openpiton hw/tile/mw_piton_core.v). A Microwatt core issues 32-bit real
# addresses; the tile widens them to 40-bit P-Mesh physical addresses:
#
#     core 0xCFxx_xxxx               ->  PA 0x98_00xx_xxxx   (L1.5 CSRs: IPI)
#     core 0xC000_0000..0xCEFF_FFFF  ->  PA 0xFF_F000_0000 + (core & 0x0FFF_FFFF)
#     anything else                  ->  PA = core
#
# The DTS is written in CORE addresses. A device the core cannot reach
# through that map is refused, never silently emitted at a wrong address.

import os
import sys
import time

# (pa_base, pa_limit_exclusive, core_base), mirrors mw_piton_core.v
CORE_WINDOWS = (
    (0x98_0000_0000, 0x98_0100_0000, 0xCF00_0000),
    (0xFF_F000_0000, 0xFF_FF00_0000, 0xC000_0000),
    (0x0, 0xC000_0000, 0x0),
)

# mw_xics (OPN-P2.15, rr-openpiton hw/ip/mw_xics): ICP at +0, 16 bytes per CPU
# (XIRR_POLL, XIRR, -, MFRR: icp-native.c struct icp_ipl); ICS at +0x1000 with
# its XIVEs at +0x800 + 4n (ics-native.c ics_native_xive), so the ICS reg must
# span 0x1000, not upstream microwatt.dts's 0x100. Source 0x10 is the UART.
XICS_WINDOW = 0x2000
XICS_ICS_OFFSET = 0x1000
XICS_UART_SOURCE = 0x10
XICS_CONVERTED_NCPUS = (1, 2)   # rr-openpiton hw/ip/mw_xics/rtl/mw_xics.v (OPN-P2.15)

# devices*.xml entries that are chipset plumbing, not something Linux drives
NOT_FOR_LINUX = ("chip", "iob")


def pa_to_core(name, base, length):
    """The core address of [base, base + length), or ValueError."""
    for pa_base, pa_limit, core_base in CORE_WINDOWS:
        if pa_base <= base and base + length <= pa_limit:
            return core_base + (base - pa_base)
    raise ValueError(
        "powerlib: device '%s' at PA 0x%x (+0x%x) is not reachable through the "
        "Microwatt core address map (mw_piton_core.v); windows: %s"
        % (name, base, length,
           ", ".join("PA 0x%x-0x%x" % (b, l - 1) for b, l, _ in CORE_WINDOWS)))


def _cells2(v):
    return "0x%08x 0x%08x" % (v >> 32, v & 0xFFFFFFFF)


# The core's ISA features, for a CONFIG_PPC_DT_CPU_FTRS kernel. Without this
# node the kernel falls back to matching the PVR, which has no Microwatt entry,
# so CPU_FTR_ARCH_300 stays clear and mmu_pid_bits stays 0. PRTB_SIZE_SHIFT - 12
# then wraps in the partition-table entry and the first fetch after the MMU is
# enabled takes an ISI (rr-openpiton OPN-T2.1, observed on silicon). The feature
# list is the kernel's own arch/powerpc/boot/dts/microwatt.dts (linux 6.6).
CPU_FEATURES_NODE = '''
        ibm,powerpc-cpu-features {
            display-name = "Microwatt";
            isa = <3000>;
            device_type = "cpu-features";
            compatible = "ibm,powerpc-cpu-features";

            mmu-radix {
                isa = <3000>;
                usable-privilege = <2>;
            };

            little-endian {
                isa = <2050>;
                usable-privilege = <3>;
                hwcap-bit-nr = <1>;
            };

            cache-inhibited-large-page {
                isa = <2040>;
                usable-privilege = <2>;
            };

            fixed-point-v3 {
                isa = <3000>;
                usable-privilege = <3>;
            };

            no-execute {
                isa = <2010>;
                usable-privilege = <2>;
            };

            floating-point {
                hwcap-bit-nr = <27>;
                isa = <0>;
                usable-privilege = <3>;
            };
        };
'''


def gen_power_dts(devices, nCpus, cpuFreq, timeBaseFreq, periphFreq, cache,
                  timeStamp="", model="openpiton-microwatt"):
    """The DTS text. `cache` holds the Microwatt core's cache geometry:
    icache_line, icache_lines, icache_ways, dcache_line, dcache_lines,
    dcache_ways. The *line* sizes become i-/d-cache-block-size, which Linux
    uses as the dcbz / icbi / dcbst step: they must equal the core's real line
    sizes (a larger d-cache-block-size makes clear_page leave memory dirty)."""
    assert nCpus >= 1
    known = ("mem", "uart", "mw_xics") + NOT_FOR_LINUX
    unknown = [d["name"] for d in devices if d["name"] not in known]
    if unknown:
        raise ValueError("powerlib: no device-tree mapping for %s; add one to "
                         "gen_power_dts before listing it in devices*.xml" % unknown)

    mems = [d for d in devices if d["name"] == "mem"]
    uarts = [d for d in devices if d["name"] == "uart"]
    xics = [d for d in devices if d["name"] == "mw_xics"]
    if len(xics) > 1:
        raise ValueError("powerlib: at most one 'mw_xics' device, got %d" % len(xics))
    if xics and xics[0]["length"] < XICS_WINDOW:
        raise ValueError("powerlib: mw_xics window 0x%x is smaller than the 0x%x its "
                         "ICS (+0x1000, XIVEs at +0x800) needs" % (xics[0]["length"], XICS_WINDOW))
    if xics and nCpus not in XICS_CONVERTED_NCPUS:
        # rr-openpiton mw_xics.v instantiates one converted netlist per CPU
        # count (hooks convert_mw_xics / convert_mw_xics2) and refuses others
        raise ValueError("powerlib: mw_xics is converted for %s CPUs, the config has %d"
                         % (" or ".join(str(n) for n in XICS_CONVERTED_NCPUS), nCpus))
    if len(mems) != 1:
        raise ValueError("powerlib: expected exactly one 'mem' device, got %d" % len(mems))

    s = '''// DTS generated by powerlib.gen_power_dts from the OpenPiton devices*.xml
// OpenPiton + Microwatt. Addresses are Microwatt CORE addresses.
// %s

/dts-v1/;

/ {
    #address-cells = <2>;
    #size-cells = <2>;
    model = "%s";
    compatible = "microwatt-soc";
''' % (timeStamp, model)

    if uarts:
        s += '''
    aliases {
        serial0 = &UART0;
    };

    chosen {
        stdout-path = &UART0;
    };
'''

    mem = mems[0]
    core = pa_to_core("mem", mem["base"], mem["length"])
    s += '''
    memory@%x {
        device_type = "memory";
        reg = <%s %s>;
    };
''' % (core, _cells2(core), _cells2(mem["length"]))

    c = cache
    s += '''
    cpus {
        #address-cells = <1>;
        #size-cells = <0>;
''' + CPU_FEATURES_NODE
    for k in range(nCpus):
        s += '''
        PowerPC,Microwatt@%d {
            device_type = "cpu";
            reg = <%d>;
            status = "okay";
            64-bit;
            general-purpose;
            clock-frequency = <%d>;
            timebase-frequency = <%d>;
            ibm,chip-id = <0>;
            ibm,ppc-interrupt-server#s = <%d>;
            i-cache-block-size = <%d>;
            i-cache-size = <%d>;
            i-cache-sets = <%d>;
            d-cache-block-size = <%d>;
            d-cache-size = <%d>;
            d-cache-sets = <%d>;
            reservation-granule-size = <%d>;
        };
''' % (k, k, cpuFreq, timeBaseFreq, k,
       c["icache_line"], c["icache_line"] * c["icache_lines"] * c["icache_ways"], c["icache_ways"],
       c["dcache_line"], c["dcache_line"] * c["dcache_lines"] * c["dcache_ways"], c["dcache_ways"],
       c["dcache_line"])
    s += '''    };
'''

    for i, u in enumerate(uarts):
        core = pa_to_core("uart", u["base"], u["length"])
        # Only the 8 byte-wide 16550 registers are declared (reg-shift 0, as
        # riscvlib: OpenPiton's UART16550 is modified to ns16550 spacing); the
        # xml's length is the chipset's decode window, which is larger.
        # interrupts = <source flags>: flags LSB 1 = level (xics_host_xlate);
        # the 16550 interrupt is a level. Without XICS the UART is polled.
        irq = ("""
        interrupt-parent = <&ICS>;
        interrupts = <0x%x 0x1>;""" % XICS_UART_SOURCE) if xics else ""
        s += '''
    UART%d: serial@%x {
        device_type = "serial";
        compatible = "ns16550";
        reg = <%s %s>;
        reg-shift = <0>;
        reg-io-width = <1>;
        clock-frequency = <%d>;
        current-speed = <115200>;%s
    };
''' % (i, core, _cells2(core), _cells2(8), periphFreq, irq)

    for x in xics:
        core = pa_to_core("mw_xics", x["base"], x["length"])
        ics = core + XICS_ICS_OFFSET
        # one ICP reg entry per interrupt server (icp_native_init_one_node
        # refuses a count mismatch)
        icp_regs = " ".join("%s 0x0 0x10" % _cells2(core + 0x10 * k) for k in range(nCpus))
        s += '''
    interrupt-controller@%x {
        compatible = "openpower,xics-presentation", "ibm,ppc-xicp";
        ibm,interrupt-server-ranges = <0x0 0x%x>;
        reg = <%s>;
    };

    ICS: interrupt-controller@%x {
        compatible = "openpower,xics-sources";
        interrupt-controller;
        interrupt-ranges = <0x%x 0x10>;
        reg = <%s %s>;
        #address-cells = <0>;
        #size-cells = <0>;
        #interrupt-cells = <2>;
    };
''' % (core, nCpus, icp_regs, ics, XICS_UART_SOURCE, _cells2(ics), _cells2(XICS_ICS_OFFSET))

    s += '''};
'''
    return s


def main(argv=None):
    """powerlib.py <out.dts> <cache geometry as k=v ...>, environment as pyhp."""
    import pyhplib
    argv = sys.argv[1:] if argv is None else argv
    out, kv = argv[0], dict(a.split("=", 1) for a in argv[1:])
    cache = {k: int(kv[k]) for k in ("icache_line", "icache_lines", "icache_ways",
                                      "dcache_line", "dcache_lines", "dcache_ways")}
    sysFreq = int(os.environ.get("CONFIG_SYS_FREQ", "50000000"))
    dts = gen_power_dts(pyhplib.ReadDevicesXMLFile(), pyhplib.PITON_NUM_TILES,
                        sysFreq, sysFreq, sysFreq, cache,
                        timeStamp=os.environ.get("POWERLIB_TIMESTAMP",
                                                 time.strftime("%b %d %Y %H:%M:%S")))
    with open(out, "w") as f:
        f.write(dts)


if __name__ == "__main__":
    main()
