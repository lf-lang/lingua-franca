This document includes information about supported operating systems and external dependencies that are expected to be manually installed by the user.


## Development Requirements

### **Supported Operating Systems**
|                              | C  | C++ | Python | TypeScript |
|------------------------------|----|-----|--------|------------|
| Ubuntu 20.04                 | Y  | Y   | Y      | Y          |
| MacOS 10.15 Catalina         | Y  | Y   | Y      | Y          |
| MacOS 11 Big Sur             | Y  | Y   | Y      | Y          |
| Windows 10 1909<sup>†</sup>  | Y  | Y   | Y      | Y          |
| Windows 10 2004<sup>†</sup>  | Y  | Y   | Y      | Y          |

<sup>†</sup> **Experimental:** Windows support is still considered experimental. You can track our Windows support progress [here](https://github.com/icyphy/lingua-franca/issues?q=is%3Aissue+is%3Aopen+label%3AWindows).


### **Dependencies**
 - Java >= 11

In order to develop the core Lingua Franca, the only basic requirement is to have Java >= 11. 
You should be able to download and run the provided Eclipse development project (see [Downloading and Building](https://github.com/icyphy/lingua-franca/wiki/Downloading-and-Building)). 

Moreover, you should be able to use the provided Eclipse IDE products located at [](). Any particular plugin or additional dependencies are managed by the Eclipse IDE product itself. The Eclipse IDE product will give you the ability to write a Lingua Franca program (see [Language-Specification](https://github.com/icyphy/lingua-franca/wiki/Language-Specification)), generate synthesized diagrams for your program (see [Diagrams](https://github.com/icyphy/lingua-franca/wiki/Diagrams)), and generate code in any of the supported target languages.


## Target Language Requirements
Each target language in Lingua Franca has a set of requirements to compile the generated code. Compiling the generated code is generally automatically done in the Eclipse IDE and using the `lfc` command line tool. This default behavior can be disabled using the [no-compile](https://github.com/icyphy/lingua-franca/wiki/target-specification#no-compile) target property or by using the `-n` argument for the `lfc` command line tool (see [Command Line Tools](https://github.com/icyphy/lingua-franca/wiki/Command-Line-Tools)). 

Here, we list the supported platforms for compiling and creating a binary as well as the set of requirements for each target:

### Supported Operating Systems
|                      | C             | C++ | Python | TypeScript |
|----------------------|---------------|-----|--------|------------|
| Ubuntu 20.04         | Y             | Y   | Y      | Y          |
| MacOS 10.15 Catalina | Y             | Y   | Y      | Y          |
| MacOS 11 Big Sur     | Y             | Y   | Y      | Y          |
| Windows 10 1909      | Y<sup>‡</sup> | Y   | N      | N          |
| Windows 10 2004      | Y<sup>‡</sup> | N   | N      | N          |

<sup>‡</sup> Requires WSL version 1 or version 2


### Dependencies

**C:**
  - gcc >= 7
  - **Windows Only:** Windows Subsystem for Linux (WSL) version 1 or 2 with gcc installed - see https://docs.microsoft.com/en-us/windows/wsl/install-win10.
  - **Programs using Protocol Buffers:** protoc-c 1.3.3 - see https://github.com/icyphy/lingua-franca/wiki/Protobufs.

**C++:**
 - g++ >= 7 or MSVC >= 14.20 - 1920 (Visual Studio 2019)
 - CMAKE >= 3.16

**Python:**
 - Python >= 3.6
 - pip >= 20.0.2
 - setuptools >= 45.2.0-1

**TypeScript:**
  - npm >= 6.14.4
  - protoc-c 1.3.3 (for using protobuf and data serialization - see https://github.com/icyphy/lingua-franca/wiki/Protobufs)