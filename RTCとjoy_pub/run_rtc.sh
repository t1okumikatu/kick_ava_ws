#!/bin/bash
# 1. 現在のディレクトリに移動するように変更
cd "$(dirname "$0")"

# 2. 実行権限を付与して起動
chmod +x ./go2rtc_linux_amd64
./go2rtc_linux_amd64