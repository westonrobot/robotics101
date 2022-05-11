# Lecture 2 handout

NOTE: if you are reading this is a plain text editor, ignore the lines with ```

This folder consists of 2 items

1. get-pip.py
2. python_basics.ipynb

## Getting pip
To get pip installed with your python2, navigate to this folder in your terminal and run

```bash
python2 get-pip.py
```

## To run the python basics notebook
1. Install pip (see above)
2. Install jupyter
   
   ```bash
    python2 -m pip install jupyter
   ```
3. Run jupyter notebook

    ```bash
    python2 -m jupyter notebook
    ```

4. Open a browser (if none was opened automatically) and go to localhost:8888
5. Navigate to the python_basics.ipynb and open it.
6. Documentation on how to modify/run code in jupyter noteboook: https://jupyter-notebook.readthedocs.io/en/latest/

## Troubleshooting

1. If the installation prompts you that the install directory is not on path, add the line below to your ~/.bashrc file.

    PATH=~/.local/bin:$PATH