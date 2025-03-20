# boot.py -- run on boot-up
import network
import time
import ntptime
import utime
from machine import Pin, SoftI2C, SPI
from umqtt.simple import MQTTClient
import gc
import BME280
from bh1750 import BH1750
import urequests
from MQ9 import MQ9
from sdcard import SDCard
import os
import neopixel
from ina219 import INA219
from DS3231 import DS3231

# Configurar NeoPixel
pin = Pin(12, Pin.OUT)
np = neopixel.NeoPixel(pin, 1)

####################### Configurar Wi-Fi ##########################
ssid = "Teste"  # Substitua pelo nome da sua rede
password = "JWTYUIO"  # Substitua pela senha da sua rede
##################################################################

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)
count = 0

while not wlan.isconnected():
    print('Conectando ao WiFi...')
    time.sleep(3)
    count += 1
    if count == 5:
        print('Número máximo de tentativas atingido. Seguindo modo on Premisse')
        break

if wlan.isconnected():
    print('Conexão estabelecida:', wlan.ifconfig())  # Mostra as informações de rede
else:
    print('Falha ao conectar após 5 tentativas. Seguir para modo on premisse')

# Configurar o servidor NTP
ntptime.host = 'a.st1.ntp.br'  # Servidor NTP de São Paulo

# Sincronizar com o servidor NTP e ajustar o fuso horário
try:
    print('Sincronizando com o servidor NTP...')
    ntptime.settime()
    print('Sincronização bem-sucedida!')
except Exception as e:
    print('Falha na sincronização:', e)

# Ajustar o horário para o fuso horário de São Paulo (UTC-3)
current_time = utime.localtime()
adjusted_time = utime.mktime(current_time) - 3 * 3600
adjusted_local_time = utime.localtime(adjusted_time)

formatted_time = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
    adjusted_local_time[0], adjusted_local_time[1], adjusted_local_time[2],
    adjusted_local_time[3], adjusted_local_time[4], adjusted_local_time[5]
)

print('Data e Hora Atual:', formatted_time)

########################## RTC ############################
# Configura os pinos I2C com frequência de 10.000 Hz
i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=10000)
# Inicializa o DS3231
rtc = DS3231(i2c)

if wlan.isconnected():
    rtc.set_time(adjusted_local_time[0], adjusted_local_time[1], adjusted_local_time[2],
                 adjusted_local_time[3], adjusted_local_time[4], adjusted_local_time[5]
    )
else:
    pass

#########################################################

SHUNT_OHMS = 0.1
# Inicializar e configurar o INA219
ina = INA219(SHUNT_OHMS, i2c)

# Configuração do SPI e do cartão SD
SD_CS_PIN = 5  # Exemplo: pino CS do SD conectado ao pino 5

spi = SPI(1, baudrate=1000000, polarity=0, phase=0, sck=Pin(18), mosi=Pin(23), miso=Pin(19))  # Configuração do barramento SPI com baudrate reduzido
sd = SDCard(spi, Pin(SD_CS_PIN))
sensor = MQ9(pinData=34, baseVoltage=3.3)
vfs = os.VfsFat(sd)
os.mount(vfs, "/sd")

print("Calibrating")
sensor.calibrate()
print("Calibration completed")
print("Base resistance:{:.2f}".format(sensor._ro))
gc.collect()

counter = 0
RPM = 0
speedwind = 0
period = 5000
radius = 111
pi = 3.14159265

############################# GRAVADO CARTAO SD #############################
def read_and_write_data(t, h, p, l, g, a, vn, vl, r, det):
    global sd
    try:
        dataEHora = str(det) + "\n"
        temp = "Temperatura:" + str(t) + "\n"
        hum = "Umidade:" + str(h) + "\n"
        pres = "Pressão:" + str(p) + "\n"
        luz = "Luz:" + str(l) + "\n"
        gas = "Gás﹕:" + str(g) + "\n"
        ar = "Ar:" + str(a) + "\n"
        vento = "Velocidade do vento:" + str(vn) + "\n"
        volt = "Voltagem:" + str(vl) + "\n"
        rpm = "Rpm:" + str(r) + "\n"

        with open('/sd/echos.txt', 'a') as f:
            f.write("\n")
            f.write("Parnamirim")
            f.write(dataEHora)
            f.write(temp)
            f.write(hum)
            f.write(pres)
            f.write(luz)
            f.write(gas)
            f.write(ar)
            f.write(vento)
            f.write(volt)
            f.write(rpm)
            f.write("\n")
        print("Dados gravados com sucesso")
        np[0] = (0, 255, 0)
        np.write()
    except Exception as e:
        np[0] = (255, 0, 0)
        np.write()
        print("Erro ao gravar dados:", e)

def windvelocity():
    global counter
    speedwind = 0
    counter = 0
    Anemometro = Pin(35, Pin.IN)
    Anemometro.irq(trigger=Pin.IRQ_RISING, handler=addcount)
    startTime = time.ticks_ms()
    while time.ticks_ms() < startTime + period:
        pass

def RPMcalc():
    global RPM
    RPM = (counter * 60) / (period / 1000)

def SpeedWind():
    global speedwind
    speedwind = (((4 * pi * radius * RPM) / 60) / 1000) * 3.6

def addcount(pin):
    global counter
    counter += 1

######################## CONEXAO MQTT ######################
if wlan.isconnected():
    servidor = 'test.mosquitto.org'
    topico = 'EstacaoMetIFPE'
    client = MQTTClient('NodeMCU', servidor, 1883)
else:
    pass

try:
    while True:
        sd.init_card(1320000)
        windvelocity()
        RPMcalc()
        SpeedWind()

        bme = BME280.BME280(i2c=i2c)
        bh1750 = BH1750(0x23, i2c=i2c)

        temp = bme.temperature
        hum = bme.humidity
        pres = bme.pressure
        light = bh1750.measurement
        temperature_val = float(temp[:-1])
        humidity_val = float(hum[:-1])
        pressure_val = float(pres[:-3])
        light_val = int(light)
        wind_val = int(speedwind)

        rtc_datetime = rtc.get_time()
        print(f" TESTE Horario RTC {rtc_datetime}")

        dataEHora = "{:04d}/{:02d}/{:02d} {:02d}:{:02d}:{:02d}".format(rtc_datetime[0]+750, rtc_datetime[1], rtc_datetime[2],rtc_datetime[3], rtc_datetime[4], rtc_datetime[5])

        conteudo = '{"Temperatura":' + str(temperature_val) + ', "Umidade":'+str(humidity_val)+ ', "Pressao":'+str(pressure_val)+', "Luz":'+str(light_val)+', "Gas":'+str("{:.1f}".format(sensor.readCarbonMonoxide()))+', "Rpm":'+str(RPM)+', "Ar":'+str("{:.1f}".format(sensor.readLPG()))+', "Volt":'+str(ina.voltage())+', "Vento":'+str(wind_val)+'}'

        print("\n")
        print("*****************************TESTE DE SAIDA************************************")
        print(f"Teste saida IPC {adjusted_local_time}")
        print('Data e Hora Atual pelo IPC:', formatted_time)
        print(conteudo)
        print(rtc_datetime)
        print(dataEHora)
        print("*******************************************************************************\n")

        read_and_write_data(str(temperature_val), str(humidity_val), str(pressure_val), str(light_val), str("{:.1f}".format(sensor.readCarbonMonoxide())), str("{:.1f}".format(sensor.readLPG())), str(wind_val), str(ina.voltage()), str(RPM), str(dataEHora))

        if wlan.isconnected():
            print("Publicando no servidor MQTT")
            try:
                client.connect()
                client.publish(topico.encode(), conteudo.encode())
                client.disconnect()
                print("Envio realizado.")
                gc.collect()
                time.sleep(30)
            except Exception as e:
                print(f"Erro de conexao: {e}")
        else:
            pass
except KeyboardInterrupt:
    wlan.disconnect()
    wlan.active(False)
    print("Fim.")