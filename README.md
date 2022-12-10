# Linux on M5Stack

Linux (RISC-V RV32IMA) runs on M5Stack (ESP32;Xtensa LX6).

RISC-V emulator is based on https://github.com/cnlohr/mini-rv32ima

Linux (RISC-V RV32IMA) を M5Stack (ESP32;Xtensa LX6) で動かします。

RISC-V エミュレータ部分は https://github.com/cnlohr/mini-rv32ima をもとにしています。


## How to execute / 実行方法

 1. Prepare microSD/microSDHC/(microSDXC). Place `ram.img` and `boot.bin` in it's root. Some microSD has compatibility issue, try some cards. Files are available in release page. microSDXC should be okay but not tested.
 2. Compile with PlatformIO in VSCode.
 3. Burn the firmware.
 4. Run.
    1. Show banner.
    2. Press center button.
    3. Booting Linux. Wait about 5 minutes.
    4. Login with `root` without password. You can input with Keyboard FACE or Serial. The shell will appear after 1 minutes.

.

 1. microSD/microSDHC/microSDXCを用意し、`ram.img`と`boot.bin`をルートに配置します。カードによっては互換性の問題があるので、いくつか試してください。これらのファイルはリリースページにあります。microSDXCは利用可能なはずですが、テストされてはいません。
 2. VSCodeのPlatformIOでコンパイルします。
 3. ファームウェアを転送します。
 4. 実行します。
    1. バナーが表示されます。
    2. M5Stack本体の中央のボタンを押します。
    3. 約5分をかけてLinuxが起動します。
    4. `root`でログインします。パスワードはありません。文字入力はFACESのキーボードとシリアル通信が利用できます。約1分でシェルが起動します。

## Linuxイメージをビルドする

`boot.img`は、LinuxカーネルとRAMディスクが収められたメモリイメージです。起動時にRAMの先頭へコピーされます。

このイメージはBuildrootによりビルドできます。https://github.com/cnlohr/mini-rv32ima のBuildrootを利用してください。なおカーネルコンフィグで `CONFIG_HZ=100` としたほうが実行速度は上がるようです。


## 実装メモ / note about implementation

 - エミュレーションするCPUは、RISC-V RV32IMA : 32ビットの基本命令、乗算命令、アトミック命令
 - RAM: RISC-Vマシンとしては16MB。ESP32のハードウェアとしてはSRAM 520KB。
 - RAMはSDカード上の16MBのファイルが実態で、SRAM上にキャッシュが確保されています。PSRAMは利用していません（初代M5StackにはPSRAMは実装されていません）。
 - キャッシュは1024バイトのブロックを単位として、224個が確保されています。LRU (Least recently used) アルゴリズムでブロックは再利用されます。ブロックのリストは挿入ソートで常にソートされ、二分探索で該当ブロックを探します。ブロックサイズと個数は実測により決定されました。
 - 命令フェッチとデータアクセスのたびにキャッシュの検索(と必要ならばSDカードアクセス)が発生するため、非常に遅いです。
 - RISC-Vエミュレータの本体は `src/mini-rv32ima.h` で、これはCNLohrによるコードをそのまま利用しています。

## License

MIT License, Refer License file.

Copyright (c) 2022 verylowfreq

Copyright (c) 2022 CNLohr

