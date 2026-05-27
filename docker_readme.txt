# ESP32-S3 Micro-ROS Terminal Setup (N16R8)

Полное руководство по настройке среды разработки, сборке и прошивке 6DOF терминала.

## 1. Подготовка папок на Хосте (Ubuntu 24.04)
Выполните в обычном терминале Ubuntu:
```bash
mkdir -p ~/esp
cd ~/esp

# Клонируем компонент micro-ROS
git clone --recursive https://github.com

# Копируем тестовый проект
cp -r micro_ros_espidf_component/examples/int32_publisher mros_test

2. Запуск Docker-контейнераСоздаем контейнер с доступом к USB и локальной 
папке ~/esp:bash

docker run --name micro-ros-jazzy-build \
  --privileged \
  -e HOME=/tmp \
  -v /dev:/dev \
  -v ~/esp:/host_esp \
  -it espressif/idf:release-v5.2 bash

Если контейнер создает чтото- снять потом права так - sudo chown -R $USER:$USER ~/esp


-u $(id -u):$(id -g): заставляет контейнер использовать ваши права доступа, так что файлы на хосте останутся «вашими».

## Смена рабочей директории

Если нужно сменить путь к проекту на хосте:
1. Остановить и удалить текущий контейнер:
   docker rm -f micro-ros-node
2. Запустить новый с измененным флагом -v:
   
   wifi
   docker run -it --rm   --net=host   -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp   microros/micro-ros-agent:jazzy   udp4 --port 8888
   
   usb(uart)
   docker run -it --rm   --privileged   -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp   microros/micro-ros-agent:jazzy  serial --dev /dev/ttyACM0 -b 115200
## Как работает файловая связь (-v)

Параметр -v ~/esp:/host_esp связывает папки:
- Хост (Ubuntu): ~/esp (здесь редактируем код)
- Контейнер: /host_esp (здесь запускаем idf.py build)

Изменения в одной папке мгновенно отражаются в другой.

Для повторного входа позже: 
docker start micro-ros-espidf-component-test && docker exec -it micro-ros-espidf-component-test bash


Настройка окружения (Внутри Docker)
Выполнять при каждом входе в контейнер:
source $IDF_PATH/export.sh
pip3 install colcon-common-extensions catkin_pkg lark empy
export CCACHE_DIR=/host_esp/lcd_test_official/.ccache

Откройте файл ~/esp/mros_test/CMakeLists.txt на вашей основной системе (Ubuntu) и приведите его к такому виду:

cmakecmake_minimum_required(VERSION 3.16)

# Важно: путь должен быть внутриконтейнерный!
set(EXTRA_COMPONENT_DIRS /host_esp/micro_ros_espidf_component)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mros_test)

4. Конфигурация проекта (Menuconfig)Перейдите в папку проекта:bashcd /host_esp/mros_test
idf.py set-target esp32s3
idf.py menuconfig

ОБЯЗАТЕЛЬНЫЕ НАСТРОЙКИ (Критично для N16R8):
Micro-ROS Settings:WiFi Configuration: Введите SSID и Password.Agent IP:Введите IP агента (узнать через hostname -I на хосте).
Зайдите в idf.py menuconfig.Перейдите в Component config -> ESP32S3-Specific.
Найдите Support for external, SPI-connected RAM (должно быть включено [*]).
Зайдите внутрь этого меню (SPI RAM config):Mode (Type) of SPI RAM: 
Измените с Quad на Octal.Set RAM clock speed: Поставьте 80MHz (стандарт для стабильности).
Важно для 16MB Flash:Вернитесь в главное меню.Serial flasher config -> Flash size: 
Выберите 16 MB.Flash SPI mode: 
Выберите OPI (если чип поддерживает Octal Flash) или оставьте DIO/QIO, если сомневаетесь.


idf.py build

# Прошивка и монитор (замените порт на свой, например /dev/ttyACM0)
idf.py -p /dev/ttyACM0 flash monitor

6. Запуск Агента (В новом терминале Ubuntu)Без агента ESP32 не подключится 
к ROS 2:bashdocker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888

7. Проверка связиВ отдельном терминале Ubuntu после появления "Connected" в логах:bashros2 topic list
ros2 topic echo /freertos_int32_publisher

8. Super clean build:

rm -rf components/micro_ros_espidf_component/micro_ros_src/build \
       components/micro_ros_espidf_component/micro_ros_src/install \
       components/micro_ros_espidf_component/micro_ros_src/log

idf.py fullclean
idf.py build