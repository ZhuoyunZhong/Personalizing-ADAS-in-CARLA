# simplemotion-drive
A Simple Motion v2 drive binding for python for steering torque control

### How to install

```bash
$ python3 setup.py install
```

### Run

```bash
$ python3 tests/test.py
```

EVERYTHING TESTED ON **PYTHON 3** , please use `python3` and `pip3`.

# Carla Example
Install the Python API of Carla (egg file inside `Carla/PythonAPI/carla/dist`)

### Step 1
Download and install Carla

```console
$ wget http://dist.carla.org/carla-0.9.8/pool/main/c/carla/carla_0.9.8-1_amd64.deb
$ dpkg -i carla_0.9.8-1_amd64.deb
$ sudo apt install -f
```


### Step 2
Run Carla
```bash
/opt/carla/bin/CarlaUE4.sh -quality-level=Low -fps=20
```

### Step 3
Run carla control
```bash
$ python3 carla_control.py --agent Basic
```

----

If "import carla" is not found, follow the steps below:

download and install `easy_install`
```console
$ sudo apt-get install python-setuptools python-dev build-essential 
```


using easy_install, install carla egg
```console
$ easy_install /opt/carla/PythonAPI/carla/dist/carla-0.9.8-py3.5-linux-x86_64.egg
```
