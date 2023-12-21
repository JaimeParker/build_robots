# docker

**project**

a docker project (python for example):

* dockerfile
* requirements.txt
* src (code file)

**dockerfile**

```dockerfile
FROM python:3.9.7
ADD . /ppo_eval
WORKDIR /ppo_eval
RUN pip install -r requirements.txt
ENV PATH=$PATH:/ppo_eval/ppo_eval
ENV PYTHONPATH=/ppo_eval/ppo_eval
CMD ["python", "./ppo_eval/evaluate/evaluate_async.py"]
```

**build**

firstly, cd your root path of project.

```shell
sudo docker build -t docker_name:tag . --platform linux/arm64/v8
```

`--platform` is not necessary.

with clear one, you can use

```shell
sudo docker build -t docker_name:tag .
```

tag will be latest if no tag specified.

**run**

```shell
sudo docker run ppo_docker:arm64 --platform linux/arm64/v8
```

platform isn't necessary.

**save**

save a build docker image as `.tar` file, for transferring to another machine

```shell
sudo docker save -o saved_docker_name.tar docker_name:tag
```

**load**

```shell
sudo docker load -i xxx.tar
```

**delete**

```shell
sudo docker rmi -f id
```

id can be accessed from `docker images`.s

**about qemu**

[arm环境下运行x86的docker](https://juejin.cn/s/arm%E7%8E%AF%E5%A2%83%E4%B8%8B%E8%BF%90%E8%A1%8Cx86%E7%9A%84docker)
