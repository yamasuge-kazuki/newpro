# Cansat 新プロ 16期 

## 1. I2CとSerialを有効化する

```bash
raspi-config
> Interface から I2CとSerialを有効化する [参考](https://www.y2c.co.jp/i2c-r/raspberrypi/i2c-enable/)
```

## 2. 必要なライブラリをインストールする

```bash
sudo apt-get install -y i2c-tools python3-smbus
```
## 3. リポジトリをクローンする

```bash
git clone https://github.com/yamasuge-kazuki/newpro.git
cd newpro
```

## 4. Pythonの仮想環境を作成する

```bash
python -m venv env
source ./env/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

