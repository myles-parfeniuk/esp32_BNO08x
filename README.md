<a name="readme-top"></a>


# Running Example Projects
1. Clone an instance of the repo.   
   ```sh
   git clone https://github.com/myles-parfeniuk/esp32_BNO08x.git
   ```

2. Checkout the examples branch.  
   ```sh
   cd esp32_BNO08x
   git checkout examples
   ```

3. Initialize submodules (each example project contains a submodule pointing to the main branch.)  
    ```sh
    git submodule update --init --recursive
    ```

4. Build any of the projects contained within the examples branch. If using the esp-idf VScode extension, open the desired project folder within vs-code. 
    ```sh
    idf.py fullclean
    idf.py build flash monitor
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## License

Distributed under the MIT License. See `LICENSE.md` for more information.
<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Contact

Myles Parfeniuk - myles.parfenyuk@gmail.com

Project Link: [https://github.com/myles-parfeniuk/esp32_BNO08x.git](https://github.com/myles-parfeniuk/esp32_BNO08x.git)
<p align="right">(<a href="#readme-top">back to top</a>)</p>