# SO-Arm Control 

## Installation 

Install uv

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

## Setting up scripts  

To find your motorbus port, plug in your motorbus USB cable, then run: 
```
uv run find_ports.py 
```

Check control of arm by passing joint angles: 
```
uv run test_control.py 
```


## Acknowledgements 

The code in this repo was adapted from the [Le Robot](https://github.com/huggingface/lerobot) library by HuggingFace. 

