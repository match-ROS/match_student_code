# Collection of Student Code of the Institut of Assembly Technology
This repository contains all code of the students working at the Institut of Assembly Technology of the Leibniz University Hannover.

## Add yourself to the overview table
1. Fork this repository by clicking on the `fork`-button: ![ForkButton](documentation/fork_button.png). Afterwards click `Create fork`.
2. Click on the `README.md`-file and then click on the edit-button: ![EditButton](documentation/edit_button.png) 
3. Add another line to the table at the bottom. (You can copy the line of the student above for the structure)
4. Column `Id`: Increase the previous Id by 1. Please memorize the `Id`, as it will be used in future steps. (always three numbers, f.e. `007` or `069`)
5. Column `Thesis title`: Insert the title of your thesis.
6. Column `Description`: Explain in one sentence what algorithm or code you developed in your thesis.
7. Type in a meaningful commit message and commit the changes.
8. Go to chapter [Pull Request](### Pull Request) and immediatly create a pull request to add yourself to the table so other students know a specific id is occupied. After creating this pull request go to the chapter [How to add your code](## How to add your code) and perform the steps of one of the two methods.

## How to add your code
There are two methods for adding your code. The first is only using the Github-Webinterface. The second option is by cloning the repository and working on your computer. We advice to use the second method.

### First Method
1. Navigate into the `student_code`-folder.
2. Add a folder in `match_student_code/student_code` with a `README.md` file by clicking the `Add file`-button: ![AddFileButton](documentation/add_file_button.png) and select `Add new file`. 
3. Think of a name that summarizes your work/algorithm. This name is used in the next step.
4. Behind `match_student_code/student_code/` type `Id_NameOfStep10/README.md`. Insert the `Id` from the chapter [Add yourself to the overview table](## Add yourself to the overview table) and the name you thought of from `step 3`.
5. Navigate into the created folder `Id_NameOfStep3`.
6. Add all packages and additional files into the folder by clicking the `Add file`-button: ![AddFileButton](documentation/add_file_button.png) and then `Upload files`. Drag and drop all folders or files from the File Explorer into the box on the web interface to upload. (On Windows it is Windows Explorer on Ubuntu it is File Manager)
7. Type in a meaningful commit message and commit the changes.
8. Add a `README.md`-file in your folder (see `step 4`) and use the structure of the example-README-file in `000_ExampleFolder`.
9. Fill out every necessary information in the copied structure and add any more details you want to describe or use your package.
10. Type in a meaningful commit message and commit the changes.
11. To finish, please jump to the chapter [Pull Request](### Pull Request) where you create a pull request to the main repository.

### Second Method
1. Clone the repository to your PC by executing the `git clone`-command with the forked repository.
2. Think of a name that summarizes your work/algorithm. This name is used in the next step.
3. Create a folder in `match_student_code/student_code` named `Id_NameOfStep2`. Insert the `Id` from the chapter [Add yourself to the overview table](## Add yourself to the overview table) and the name you thought of from `step 2`.
4. Add all packages and additional files into the folder, f.e. by drag and drop.
5. Add a `README.md`-file in your folder (see `step 3`) and use the structure of the example-README-file in `000_ExampleFolder`.
6. Fill out every necessary information in the copied structure and add any more details you want to describe or use your package.
7. Type in a meaningful commit message and commit the changes.
8. If not already done, push the changes to your forked repository.
9. To finish, please jump to the chapter [Pull Request](### Pull Request) where you create a pull request to the main repository.

### Pull Request
To merge your changes/added files into the official match-ROS repository, we need to create a Pull Request:
1. Navigate to the start page of your forked version of the `match_student_code` repository.
2. Check if the last commit message is by you. If not, you need to go into your forked version of the `match_student_code` repository.
3. Click on the `Pull Request`-button: ![PullRequestButton](documentation/pull_request_button.png) and then `New Pull Request` on the right side.
4. Compare two things with the following picture. Firstly, does the `base repository:` and `base:` match excactly. Secondly, is your forked version of the `match_student_code` repository listed behind `head repository` and is the branch selected that you commited your code to. 

![grafik](https://user-images.githubusercontent.com/50292612/211014212-b623642f-1ab7-4cd2-b9cb-03a260362e44.png)

5. Check if all changes are listed correctly. Did you add all necessary packages? Did you add all additional Matlab or Python scripts?
6. Create the pull request by clicking on the `Create Pull Request`-button.
7. Inform your supervisor about the newly created pull request.

## Content Overview
| Id | Thesis title | Description |
| --- | --- | --- |
| [000](student_code/000_ExampleFolder/README.md) | Example Folder | This folder contains templates and examples how it should look like |
| [001](student_code/001_UltrasoundBasedLocalization/README.md) | Evaluation of an Ultrasound-based Indoor Localization System for Mobile Multi-robot Systme | In this work we developed a localization system based on the sensor fusion approach using the EKF|
| [002](student_code/002_SplinedVoronoiPlanner/README.md) | Planung und Glättung von Pfaden für Formationen nicht-holonomer mobiler Roboter | Path planning with voronoi diagrams and smoothing with quintic bezier-splines |
| [003](student_code/003_RedundancyRes/Readme.md) | Development of an Optimization Function for Redundancy Resolution while Avoiding Singularities for Redundant Mobile Manipulators | Optimization for the UR16 and MiR100/200 on basis of reduced gradient |
| [004](student_code/004_ips_sensor_fusion/README.md) | Entwicklung und Evaluierung von Konzepten zur Nutzung von Ultraschall Indoor Positioning Systemen in der mobilen Robotik | concept for fusing ultrasonic ips data with amcl and imu using kalman filters |
| [005](student_code/005_RLS-Algorithm/README.md) | Entwicklung eines Algorithmus zur Schätzung der dynamischen Parameter unbekannter Objekte in einem Multirobotersystem | In this work we developed an Recursive-Least-Squares Algorithm to estimate the dynamic parameters of an unknown object |

