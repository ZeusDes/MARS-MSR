# MARS-MSR
NASA Mars Sample &amp; Return Rover

## Installation

### 1. Create Python enviroment
#### if you are using conda (be sure that conda is activated on your terminal): 
```bash
conda env create -f environment.yml
conda activate Rover
```
#### otherwise: 
```bash
python3 -m venv myenv
source ./myenv/bin/activate
```

### 2. install the required packages
```bash
pip install -r req.txt
```

### 3. Run the code
```bash
cd code
python3 drive_rover.py
```
> ℹ️ Note: to record your run you can specify a directory name the end of the above second command.

<h3> 4. Open Roversim Simulator </h3>
    <ol>
        <li>
            Navigate to the Simulator folder and open the suitable version of the simulator.
        </li>
        <li>
            when the simuator opens choose Automonus mode.
        </li>
    </ol> 
