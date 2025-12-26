# Docker 常用命令

### 1. Docker[安装教程](https://docs.docker.com/engine/install/ubuntu/#installation-methods)

### 2. Docker常用命令

```bash
1. 从本地导入镜像
sudo docker load -i redis.tar/ubuntu

2. 从仓库拉取镜像
sudo docker pull <image_name>:latest

3. 查看镜像列表
sudo docker images

4. 从镜像创建容器
sudo docker run <IMAGE>

5. 查看所有容器
sudo docker ps -a

6. 查看正在运行的容器
sudo docker ps

7. 启动容器
sudo docker start <CONTAINER>

8. 进入正在运行的容器
sudo docker exec -it <CONTAINER_NAME> /bin/bash

9. 停止容器
sudo docker stop <CONTAINER_NAME>

10. 删除容器
sudo docker rm <CONTAINER_NAME>

11. 删除镜像
sudo docker image rm <image_name> or <image_id>
```

### 3. 导出/导入容器

- 导出容器

```bash
//查看当前运行的容器
docker ps

//导出容器，一般保存为.tar文件
/*
TODO:<container_name>为想要导出容器的名称
	 /path/to/save 为想要导出容器存放的路径
	 container_backup.tar 为导出的容器文件名
*/
docker export <container_name> -o /path/to/save/container_backup.tar

//验证导出结果
ls /path/to/save/container_backup.tar
```

- 导入本地容器

```bash
//导入打包好的容器,其中<new_image_name>为创建的新镜像名称
docker import /path/to/save/container_backup.tar <new_image_name>
```

### 4. 导出/导入镜像

- 镜像保存，以下图为例

![image](../../images/tools/image2.png)

```bash
//注：导出镜像时，最好先完全停止容器，防止导出时容器文件不完整
//查看现有容器镜像，查询需要保存的镜像ID
docker ps -a

//1.使用commit参数来保存镜像 commit命令用于将容器的当前状态保存为一个新的Docker镜像
/*
	docker commit <IMAGE ID> <IMAGE_NAME>:<TAG>
	标签<IMAGE_NAME>和<TAG>为导出时的自定义标签
*/
docker commit 3135b315b47c redis/redis-stack-server:latest

//2.使用save参数将保存的镜像保存到一个tar归档中，便于分发或备份
/*
	docker save -o <TAR_NAME> <IMAGE_NAME>:<TAG>
	-o, --output:指定输出路径
	标签<TAR_NAME>为导出的tar包名
	标签<IMAGE_NAME>:<TAG>与 1 中的标签一致
*/
docker save -o myimage.tar redis/redis-stack-server:latest

//3.验证保存的tar文件
ls -lh myimage.tar

//输入示例(参考)
-rw-r--r-- 1 user user 200M Jul 24 14:00 myimage.tar
```

- 镜像导入

```bash
//导入 1 中保存的tar包
docker load -i myimage.tar
```

### 5. 两种方法的区别

- `export`和`import`导出的是一个容器的快照，不是镜像本身，也就是说没有layer(Docker镜像是由镜像基础层-构建中间层-可变层(容器层)构成)；
- 两种导出方法都会使`dockerfile`中的workdir，entrypoint之类的所有东西都会丢失；
- 快照文件将丢弃所有的历史记录和元数据信息(即仅保存容器当时的快照状态)，而镜像存储文件将保存完整记录，单tar包体积也更大
- `docker save`保存的是镜像(image)，`docker export`保存的是容器(container)
- `docker load`用来载入镜像包，`docker import`用来载入容器包，但两者都会恢复为镜像
- `docker load`不能对载入的镜像重命名，`docker import`可以为镜像指定新名称

### 6. 示例

```shell
# 如果只需要redis，将ubuntu删除即可
sudo docker load -i redis.tar/ubuntu

# -d：表示该容器在后台运行
# --name redis：为新容器指定名称为redis
# -p 6379:6379：将主机的端口6379映射到容器的端口6379。redis的默认端口是6379，映射后可以通过主机的localhost:6379访问容器内的redis服务
# redis/redis-stack-server：运行的镜像名称
sudo docker run -d --name redis -p 6379:6379 redis/redis-stack-server
```