GrSim Installation:

1. Read INSTALL.md from [https://github.com/RoboCup-SSL/grSim/blob/master/INSTALL.md](https://github.com/RoboCup-SSL/grSim/blob/master/INSTALL.md)

For Linux, make sure that the necessary packages have been installed:

$ sudo apt-get install git build-essential cmake pkg-config qt5-default libqt5opengl5-dev libgl1-mesa-dev libglu1-mesa-dev libprotobuf-dev protobuf-compiler libode-dev libboost-dev

- Error may arise later eg.: libqt4, qt-error if these are not installed properly. Make sure that Eigen and OMPL libraries are installed.

- **Dependencies:** GrSim depends on:

- [CMake] ([https://cmake.org/](https://cmake.org/)) version 2.8+

- [OpenGL] ([https://www.opengl.org](https://www.opengl.org/))

- [Qt4 Development Libraries] ([https://www.qt.io](https://www.qt.io/)) version 4.8+

- [Open Dynamics Engine (ODE)] ([http://www.ode.org](http://www.ode.org/))

- [VarTypes Library] ([https://github.com/szi/vartypes](https://github.com/szi/vartypes))

([https://github.com/jpfeltracco/vartypes.git](https://github.com/jpfeltracco/vartypes.git))

- [Google Protobuf] ([https://github.com/google/protobuf](https://github.com/google/protobuf))

- [Boost development libraries] ([http://www.boost.org/](http://www.boost.org/)) (needed by VarTypes)

**Note:** It&#39;s necessary to compile ODE in double precision. This is default when installing the ODE binaries in Ubuntu. However, if you are compiling ODE from source (e.g on Mac OS), please make sure to enable the double precision during the configuration step: `./configure --enable-double-precision`.

1. _Note that during the installation, the correct version of vartypes is to be taken care of, at every step._There are two sources:

1. szi/vartypes : [https://github.com/szi/vartypes.git](https://github.com/szi/vartypes.git)
2. jpfeltracco/vartypes : [https://github.com/jpfeltracco/vartypes.git](https://github.com/jpfeltracco/vartypes.git)

Now, the GrSim installed from the ROBOCUP-site needs jpfeltracco/vartypes. But the KRSSG codebase will require szi/vartypesbefore compilation.

**Another crucial thing is that the installation must be completed in one go, without repetition of steps, which may result in errors; and understanding the steps is a must. So, if the installation fails once(generally), go for it again.**

- **Python 2: should be set as default**
- **ROS: should be installed with Python 2 set as default(may be changed later )**

1. Next, compile and install VarTypes from source. In the following we install VarTypes from source using git.

$ cd /tmp

$ git clone [https://github.com/jpfeltracco/vartypes.git](https://github.com/jpfeltracco/vartypes.git) :for GrSim

$ cd vartypes

$ mkdir build

$ cd build

$ cmake ..

$ make

$ sudo make install

- **This is the general process for vartypes installation in** /tmp (temporary storage folder)

Next, clone grSim into your preferred location.

$ cd /path/to/grsim\_ws

$ git clone [https://github.com/RoboCup-SSL/grSim.git](https://github.com/RoboCup-SSL/grSim.git)

$ cd grSim

Create a build directory within the project (this is ignored by .gitignore):

$ mkdir build

$ cd build

Run CMake to generate the makefiles:

$ cmake ..

Then compile the program:

$ make

The GrSim(latest version as on RoboCup-SSL) is now installed in your system. The binary is copied to the ../bin folder after a successful compilation.

- _ **POSSIBLE ERRORS:** __ **Dependencies: Check whatever dependency is missing. Can be solved by installation from the appropriate repo. Make sure all of the steps are performed in one GO, if not restart the system and repeat the process.** _

KRSSG-SSL Codebase Installation:

1. Install the _szi/vartypes_: This is required for the KRSSG codebase.

$ cd /tmp

$ git clone [https://github.com/szi/vartypes.git](https://github.com/szi/vartypes.git) **:for KRSSG Codebase**

$ cd vartypes

$ mkdir build

$ cd build

$ cmake ..

$ make

$ sudo make install

1. Proceed as:

Create a folder (in preferably /user/home), for e.g.: KRSSG

$ cd KRSSG

$ git clone [https://github.com/KRSSG/robocup](https://github.com/KRSSG/robocup)

- rename downloaded folder to &quot;src&quot; -

- Check if the libraries: Eigen and OMPL are present in /usr/include/c++ and /usr/include. If not, then make sure they are copied into these folders. Generally, they have to be copied from /usr/include into /usr/include/c++ or sometimes need to be downloaded.

IMPORTANT: Related Errors during cmake which may arise due to missing libraries: Therefore, make sure these are present before proceeding with ROS compilation(i.e. Before and During Step-3)

- **fatal error:** Eigen/Dense: No such file or directory **#include\&lt;Eigen/Dense\&gt;** : arises due to absence of the _Eigen library_ in the respective destination. Make sure that the directory Eigen/Dense is directly placed in the include directly. It should not be Eigen/Eigen/Dense or anything other than the Eigen folder itself. This generally happens when the Eigen is installed from online repo.

In BASH: $ sudo apt-get install libeigen3-dev

- **fatal error:** ompl/geometric/… : No such file or directory # **include\&lt;ompl/geometric/...\&gt;** : arises due to absence of _ompl library_in the respective destination.

Site: [https://ompl.kavrakilab.org/installation.html](https://ompl.kavrakilab.org/installation.html)

- **fatal error:** QtWidgets/QtApplication… : No such file or directory # **include\&lt;QtWidgets...\&gt;** : arises due to improper placing of the Qt directory. Make sure that the appropriate version of Qt is installed and placed in /usr/include/c++ and QtGui and QtWidgets are placed in /usr/include/c++

- All similar errors arising as : No such file or directoryto be treated in the same manner.

![](RackMultipart20200705-4-1d9hzc_html_6d40eb638874b941.png)

The contents of /usr/include/c++/ should look somewhat similar to this

(Note: qt5 is available online while qt4 isn&#39;t: It won&#39;t make a difference. It will work. Just make sure that the steps are executed in one go, if any wrong step is taken, repeat the entire installation process)

1. Now, once the repository has been cloned and renamed as src, proceed as:

1. Open the grSim folder downloaded from RoboCup-SSL and the grSim folder in the src folder.
2. Copy the _package.xml_ file from the grSim(inside src-i.e.KRSSG) into the grSim(RoboCup-SSL)
3. REPLACE the grSim in the src folder with the grSim(downloaded from RoboCup-SSL)
4. Go to the file : src/kgpkubs\_launch/scripts/ssl.shand in line 26 change:

launcher &quot;grSim&quot;&quot;rosrun grSim grsim&quot;

to ----

launcher &quot;grSim&quot;&quot;rosrun grSim grSim&quot;

grsim or grSim depending on what is present in src/grSim/bin

1. Once all these changes have been made, proceed with the INSTALLATION:

$ cd KRSSG - outside src

$ catkin\_make --pkg krssg\_ssl\_msgs

-- Remember than szi/vartypes is currently installed

$ catkin\_make //produces maximum errors; solve one by one as

mentioned before

$ source ./devel/setup.bash

**NOTE:** _Don&#39;t continue just after solving the dependency errors, make sure that the installation is done in one Go._ _So, reboot and give a fresh start to the installation_ _once all the errors as a result of_ **$ catkin\_make** _are solved._

1. Now, the szi/vartypes is currently installed. Reinstalljpfeltracco/vartypesfor grSim to be able to run.
2. Now,

$ cd src

$ chmod 777 kgpkubs\_launch/scripts/ssl.sh

$ ./kgpkubs\_launch/scripts/ssl.sh

$ python test\_role.py_-- to test the bots_

Two windows will open up. One is the top view of the game-ground. The other is a 3d-simulator. (6 Bots on either side of the Half-Way-Line)

- _In total the vartypes is installed_ _ **three** _ _times: Once before installing GrSim, once before catkin\_make of KRSSG codebase and once before running the GrSim in the codebase(final)_

1. Make sure that in the home user directory, in the hidden file: .grsim.xml : no. of bots is changed to 6: (Leads to error: UDP Datagram file sending failed)

**NOTE** : Sometimes errors may occur after catkin\_make due to overwritten files, so make sure that after solving catkin\_make errors, you reboot and continue fresh.
