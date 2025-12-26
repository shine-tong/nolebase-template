# package.xml 结构
- 包清单，位于一个`package`的根目录，与`CMakeLists.txt`处于同级目录下
- 此文件定义`package`的属性，例如：包名、版本号、作者、维护者以及对其他**catkin包**的依赖关系


### 1.  根标签

```xml
<package format="2">

</package>
```

### 2. 所需标签

```xml
<name> 包名
<verison> 版本号
<description> 内容描述
<maintriner> 维护人员
<license> 发布代码所依据的软件许可 (例如GPL/BSD/ASL)
```

- 示例

```xml
<package format="2">
  <name>foo_core</name>
  <version>1.2.4</version>
  <description>
  This package provides foo capability.
  </description>
  <maintainer email="ivana@osrf.org">Ivana Bildbotz</maintainer>
  <license>BSD</license>
</package>
```

### 3. 依赖关系

- Build Dependencies：构建依赖项，即构建所依赖的包 `<build_depend>`
- Build Export Dependencies：构建导出依赖项，即指定根据此包构建库所需的包 `<build_export_depend>`
- Execution Dependencies：执行依赖关系，即指定运行此包中的代码所需要的包 `<exec_depend>`
- Test Dependencies：测试依赖项，即指定单元测试的附加依赖项 `<test_depend>`
- Build Tool Dependencies：构建工具依赖项，即指定此包构建自身所依赖的构建工具` <buildtool_depends>`
- Documentation Tool Dependencies：文档工具依赖项，即指定此包生成文档所需的文档工具   **<doc_depend>**
- **注：必须包含一个依赖项**  

- 示例

```xml
<package format="2">
  <name>foo_core</name>
  <version>1.2.4</version>
  <description>
    This package provides foo capability.
  </description>
  <maintainer email="ivana@willowgarage.com">Ivana Bildbotz</maintainer>
  <license>BSD</license>
  <url>http://ros.org/wiki/foo_core</url>
  <author>Ivana Bildbotz</author>
  <!--指定构建工具-->
  <buildtool_depend>catkin</buildtool_depend>
  <!--依赖包-->
  <depend>roscpp</depend>
  <depend>std_msgs</depend>
  <!--构建所需的包-->
  <build_depend>message_generation</build_depend>
  <!--执行所需依赖包-->
  <exec_depend>message_runtime</exec_depend>
  <exec_depend>rospy</exec_depend>
  <!--测试依赖包-->
  <test_depend>python-mock</test_depend>
  <!--文档生成依赖包-->
  <doc_depend>doxygen</doc_depend>
</package>
```

### 4. 元包（Metapackages）

- 将多个包分组为单个逻辑包

```xml
<export>
    <metapackage />
```

- 注1  **：除必填项**`<buildtool_depends>`**依赖于catkin，**  元包只对其分组的包具有**执行依赖性**
- 注2：元包还有一个必须的`CMakeList.txt`文件

    ```cmake
    cmake_minimum_required(VERSION 2.8.3)
    project(<PACKAGE_NAME>)
    find_package(catkin REQUIRED)
    catkin_metapackage()
    ```
- 附件标签

    - 有关软件包信息的URL，通常是ros.org上的wiki页面 <url_>
    - `<author> `包的创作者