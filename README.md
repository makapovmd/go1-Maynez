# Go1 MIREA

Контейнер для задания T1 хакатона РТУ МИРЭА по Unitree Go1. Образ использует Ubuntu 22.04, ROS 2 Humble и Cyclone DDS и запускается без прав root.

## Сборка и проверка

Из папки с Dockerfile выполните:

```sh
docker buildx build --platform linux/amd64 --load -t go1-t1:local .
docker run --rm --pull never --network none --entrypoint /entrypoint.sh go1-t1:local ping
```

Для сборки нужен интернет. Контейнер запускается без доступа к сети и GPU. Команда `ping` завершится с кодом 0 и выведет один JSON-объект:

```json
{"protocol_version":"1.0","supported_tasks":[],"self_trained_rl":false,"preferred_speed_mps":null}
```

В этой версии реализован T1.
