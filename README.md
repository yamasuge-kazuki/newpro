# Cansat 新プロ 16期 

## 1. I2CとSerialを有効化する

```bash
raspi-config
> Interface から I2CとSerialを有効化する [参考](https://www.y2c.co.jp/i2c-r/raspberrypi/i2c-enable/)
```

## 2. リポジトリをクローンする

```bash
git clone https://github.com/yamasuge-kazuki/newpro.git
cd newpro
```

## 3. Pythonの仮想環境を作成する

```bash
python -m venv env
source ./env/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

## 4. 必要なライブラリをインストールする

```bash
sudo apt-get install -y i2c-tools python3-smbus
pip install pyserial
```

##  GPSの動作確認方法

```bash
sudo apt-get install screen
screen /dev/serial0 9600
```
1.真っ暗な画面が表示される場合：ラズパイがGPSを認識できていない\
2.エラーがでる場合：/dev/serial0を変えてみる　参照：https://tech-and-investment.com/raspberrypi2-5-uart-setting/ \
3.NMEAフォーマットが出力されるが、緯度経度の値が----になっている場合：GPSは認識されているがGPSが人工衛星を認識できていない。GPS受信機は屋内では人工衛星を補足できないので、壁や天井に遮られない場所に出て、しばらく放置してみよう\
4.NMEAフォーマットが出力され、緯度経度の値が妥当な場合：成功!
