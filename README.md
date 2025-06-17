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

Check control of arm by passing joint angles (for default position): 
```
uv run client.py --uid so101 --action 0.0054 -0.0069 0.0069 1.598 1.5789 0.0177
```


## Acknowledgements 

The code in this repo was adapted from the [Le Robot](https://github.com/huggingface/lerobot) library by HuggingFace. 

