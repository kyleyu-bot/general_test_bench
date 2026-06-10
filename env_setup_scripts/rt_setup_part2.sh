#!/usr/bin/env bash
set -euo pipefail

### ===== CONFIG =====
NIC="ecat0"
RT_CPU="2"
COMBINED_QUEUES="1"

echo "===== RT SETUP START ====="

### 1. Disable irqbalance
echo "[1] Disabling irqbalance..."
sudo systemctl stop irqbalance || true
sudo systemctl disable irqbalance || true

### 2. Set CPU governor to performance
echo "[2] Setting CPU governor to performance..."
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  echo performance | sudo tee "$cpu" > /dev/null
done

echo "Governor status:"
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort | uniq -c

### 3. Force NIC to a single combined queue
echo "[3] Setting $NIC to combined queue count = $COMBINED_QUEUES ..."
sudo ethtool -L "$NIC" combined "$COMBINED_QUEUES"

echo "Channel status:"
sudo ethtool -l "$NIC"

### 4. Find and pin NIC IRQs
echo "[4] Finding IRQs for NIC: $NIC"
IRQS=$(grep -i "$NIC" /proc/interrupts | awk -F: '{print $1}' | tr -d ' ')

if [[ -z "${IRQS}" ]]; then
  echo "ERROR: No IRQs found for NIC $NIC"
  exit 1
fi

echo "Found IRQs: $IRQS"

echo "[4] Pinning IRQs to CPU $RT_CPU..."
for irq in $IRQS; do
  echo "$RT_CPU" | sudo tee "/proc/irq/$irq/smp_affinity_list" > /dev/null
done

### 5. Verify IRQ affinity
echo "IRQ affinity check:"
for irq in $IRQS; do
  printf "IRQ %s -> " "$irq"
  cat "/proc/irq/$irq/smp_affinity_list"
done

echo "Current interrupt rows for $NIC:"
grep -i "$NIC" /proc/interrupts || true

echo "===== RT SETUP DONE ====="
echo "Tip: verify live placement with:"
echo "watch -n 0.5 'grep $NIC /proc/interrupts'"