# rhex_controller

#### Here we keep the gait controllers (generators) that make our rhexs alive.

## Available controllers

### RhexControllerSimple

A simple controller in which 3 of the legs are in phase with eachother and the other 3 legs are out of phase of eachother but parameters like the phase difference between the legs and the duty cycle and the size of the fast roation period to slow aswell as the values of the proportional controller and diffrential controller can be updated.

## How to compile

### Compile and install

- cd to `rhex_controller` folder
- Configure with `./waf configure --prefix=path_to_install`
- Compile with `./waf build`
- Install with `./waf install`

## How to use it in other projects

### Using the WAF build system

Add rhex_controller as an external library using the following script:

```python
#! /usr/bin/env python
# encoding: utf-8
# Konstantinos Chatzilygeroudis - 2015

"""
Quick n dirty rhex_controller detection
"""

import os
from waflib.Configure import conf


def options(opt):
	opt.add_option('--controller', type='string', help='path to rhex_controller', dest='controller')

@conf
def check_rhex_controller(conf):
	includes_check = ['/usr/local/include', '/usr/include']

	# You can customize where you want to check
	# e.g. here we search also in a folder defined by an environmental variable
	if 'RESIBOTS_DIR' in os.environ:
		includes_check = [os.environ['RESIBOTS_DIR'] + '/include'] + includes_check

	if conf.options.controller:
		includes_check = [conf.options.controller + '/include']

	try:
		conf.start_msg('Checking for rhex_controller includes')
		res = conf.find_file('rhex_controller/rhex_controller_simple.hpp', includes_check)
		conf.end_msg('ok')
		conf.env.INCLUDES_RHEX_CONTROLLER = includes_check
	except:
		conf.end_msg('Not found', 'RED')
		return
	return 1
```

Then in your C++ code you would have something like the following:

```cpp
// previous includes
#include <rhex_controller/rhex_controller_simple.hpp>

// rest of code

rhex_controller::RxexControllerSimple controller(controller_parameters, broken_legs);

// rest of code
```


## LICENSE

[CeCILL]

[CeCILL]: http://www.cecill.info/index.en.html
