# Go1 MIREA

Контейнер для заданий T1 и T2 хакатона РТУ МИРЭА по Unitree Go1. Образ использует Ubuntu 22.04, ROS 2 Humble, Cyclone DDS и MuJoCo 3.3.7.

## Сборка и проверка

Из папки с Dockerfile выполните:

```sh
docker buildx build --platform linux/amd64 --load -t go1-t2:local .
docker run --rm --pull never --network none --entrypoint /entrypoint.sh go1-t2:local ping
```

Для сборки нужен интернет. Контейнер запускается без доступа к сети и GPU. Команда `ping` завершится с кодом 0 и выведет один JSON-объект:

```json
{"protocol_version":"1.0","supported_tasks":["T2"],"self_trained_rl":false,"preferred_speed_mps":null}
```

## T2

`/entrypoint.sh T2` запускает узел, который читает `/lowcmd_bare` и публикует `/lowcmd` по `/clock` с периодом 10 мс симуляционного времени. Узел проверяет углы, ограничивает шаг каждой цели до 5° и проверяет переход по всем 655 парам из таблицы организаторов. Если проверка не подтверждает безопасность либо в команде есть NaN или Inf, предыдущая цель сохраняется. После 450 мс без принятой команды узел удерживает её с нулевыми `dq` и `tau`.

Из корня публичного симулятора на Linux:

```sh
python3 run_local.py --task T2 --headless --sim-image registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0 --participant-image go1-t2:local
```

Модель Go1, таблица пар и перечень зависимостей взяты из публичного симулятора. Сообщения `unitree_go` совпадают с версией, которую использует симулятор. [Симулятор и модель](https://github.com/Urika000/go1-simulator-public/tree/3473f086fd8a111592b4b9d822500c79f20c811a) · [Unitree ROS 2](https://github.com/unitreerobotics/unitree_ros2/tree/668d1ec5a05d1c38d3306bdca7d59f2ba3581a88)
