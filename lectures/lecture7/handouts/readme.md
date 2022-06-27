# Lecture 7 handout

NOTE: if you are reading this is a plain text editor, ignore the lines with ```

This folder consists of 1 file and 1 folder

1. python_basics.ipynb
2. assets - contains the images used by the jupyter notebook.

## Follow the instructions to install the correct version of OpenCV in your local system to ensure compatibility with the LIMO.

1. OpenCV version running on LIMO: 4.1.1
2. Install the specific version of OpenCV 
    
    ```bash
    pip install opencv-python==4.1.1.26
    ```

## To run the OpenCV basics notebook
1. Run jupyter notebook

    ```bash
    python2 -m jupyter notebook
    ```

2. Open a browser (if none was opened automatically) and go to localhost:8888
3. Navigate to the opencv_basics.ipynb and open it.
4. Documentation on how to modify/run code in jupyter noteboook: https://jupyter-notebook.readthedocs.io/en/latest/

## Troubleshooting
1. If you have previous/other manually installed (= not installed via pip) version of OpenCV installed (e.g. cv2 module in the root of Python's site-packages), remove it before installation to avoid conflicts.
2. Make sure that your pip version is up-to-date. If not run
    
    ```bash
     pip install --upgrade pip
     ```
