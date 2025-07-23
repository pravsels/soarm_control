# SO-Arm Control 

## Installation 

Install uv

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

## Setting up scripts  

To find your motorbus port, plug in your motorbus USB cable, then run: 
```
uv run find_port.py 
```

Perform homing calibration by moving arm to its mid position as shown [here](https://huggingface.co/docs/lerobot/en/so101#calibration-video). Then for each joint, move it through its full range so that we can note down min and max range.
```
uv run calibrate.py 
```

Check control of arm by passing joint angles: 
```
uv run test_control.py 
```


## Acknowledgements 

The code in this repo was adapted from the [Le Robot](https://github.com/huggingface/lerobot) library by HuggingFace. 

