#!/bin/bash

# ===================== 【配置区】所有路径改为绝对路径，仅修改这里 =====================
# 1. OpenOCD 根目录（绝对路径）
OPENOCD_ROOT="/usr/local"
# 2. DAPLink 配置文件（绝对路径）
DAPLINK_CFG="/home/qza/Horizon/User_Config/ozone_daplink.cfg"
# 3. 等待 OpenOCD 启动的时间（秒）
WAIT_TIME=2
# ==================================================================================

# 自动拼接 OpenOCD 相关路径（无需修改）
OPENOCD_EXE="${OPENOCD_ROOT}/bin/openocd"
TARGET_CFG="${OPENOCD_ROOT}/share/openocd/scripts/target/stm32h7x.cfg"

# 检查文件是否存在（避免路径错误）
if [ ! -f "$OPENOCD_EXE" ]; then
    echo -e "\033[31m[错误] 未找到 OpenOCD：${OPENOCD_EXE}\033[0m"
    exit 1
fi

if [ ! -f "$DAPLINK_CFG" ]; then
    echo -e "\033[31m[错误] 未找到 DAPLink 配置：${DAPLINK_CFG}\033[0m"
    exit 1
fi

# 启动提示
echo "正在启动 OpenOCD 调试服务器..."
echo ""
echo "连接信息："
echo " - GDB 连接端口：3333"
echo " - Telnet 连接端口：4444"
echo " - TCL 连接端口：6666"
echo ""
echo "提示：请勿关闭此窗口（或按 Ctrl+C），否则调试服务器会断开！"
echo "--------------------------------------------------------"

# 启动 OpenOCD 并直接在当前窗口输出日志
# 用 exec 可以让 Ctrl+C 完美直接退出 OpenOCD
exec "$OPENOCD_EXE" -f "$DAPLINK_CFG" -f "$TARGET_CFG"
