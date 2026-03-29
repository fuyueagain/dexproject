#!/bin/bash
# YuanClaw Loop Agent 启动脚本
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# 激活 conda 环境 (miniforge3)
source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate new-lerobot
echo "  Python: $(which python) ($(python --version 2>&1))"

# 安装依赖
pip install -q fastapi uvicorn httpx websockets python-multipart 2>/dev/null || true

cd "$PROJECT_DIR"

echo "=========================================="
echo "  YuanClaw Loop Agent"
echo "  http://0.0.0.0:8080"
echo "=========================================="
echo ""

# 启动服务
python -m loop_agent
