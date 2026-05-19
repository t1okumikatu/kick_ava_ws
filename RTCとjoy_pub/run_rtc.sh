#!/bin/bash
# 1. 現在のディレクトリに移動するように変更
# 1. RTCとjoy_pub ディレクトリに移動
cd "$(dirname "$0")/RTCとjoy_pub"
# 2. 実行権限を付与して起動
chmod +x ./go2rtc_linux_amd64
./go2rtc_linux_amd64