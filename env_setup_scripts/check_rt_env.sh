#!/usr/bin/env bash
# Read-only functional check of the realtime tuning applied by
# tune_realtime.sh / rt_setup_part2.sh.  Exit 0 iff fully tuned.
#
# Run unprivileged at every launch: the launchers only invoke env_setup.sh
# (and its sudo prompt) when this check fails.  This self-heals tuning that
# was silently skipped (NIC not up yet at boot) or reverted after the fact
# (power-profiles-daemon resetting the CPU governor, irqbalance re-enabled).
#
# Intentionally no set -e: report ALL failing items, then exit nonzero.

### ===== CONFIG (keep in sync with rt_setup_part2.sh) =====
NIC="enp47s0"
RT_CPU="2"

rc=0
fail() { echo "check_rt_env: $*" >&2; rc=1; }

# 1. All CPU governors == performance.
for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    [[ -r "$f" ]] || continue
    gov=$(<"$f")
    [[ "$gov" == "performance" ]] || fail "$f = $gov (want performance)"
done

# 2. NIC IRQs exist and are pinned to the RT CPU.
irqs=$(grep -i "$NIC" /proc/interrupts | awk -F: '{print $1}' | tr -d ' ')
if [[ -z "$irqs" ]]; then
    fail "no IRQs found for $NIC (link down or interface renamed?)"
else
    for irq in $irqs; do
        aff=$(cat "/proc/irq/$irq/smp_affinity_list" 2>/dev/null)
        [[ "$aff" == "$RT_CPU" ]] || fail "IRQ $irq affinity=${aff:-unreadable} (want $RT_CPU)"
    done
fi

# 3. irqbalance must be inactive (it rewrites IRQ affinities).
if systemctl is-active --quiet irqbalance; then
    fail "irqbalance is active"
fi

exit $rc
