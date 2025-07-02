[![CC BY-SA 4.0][cc-by-sa-shield]][cc-by-sa]

Mechanical design is licensed under a :
[Creative Commons Attribution-ShareAlike 4.0 International License][cc-by-sa].
[![CC BY-SA 4.0][cc-by-sa-image]][cc-by-sa]

[cc-by-sa]: http://creativecommons.org/licenses/by-sa/4.0/
[cc-by-sa-image]: https://licensebuttons.net/l/by-sa/4.0/88x31.png
[cc-by-sa-shield]: https://img.shields.io/badge/License-CC%20BY--SA%204.0-lightgrey.svg

Software work is licensed under a : Apache2


# AmazingHand (a.k.a AH!) Project

[![Demo](https://img.youtube.com/vi/U0TfeG3ZUto/maxresdefault.jpg)](https://www.youtube.com/watch?v=U0TfeG3ZUto)


# Amazing Hand project

Robotic hands are often very expensive and not so expressive. More dexterous often needs cables and deported actuators in forearm i.e..
Aim of this project is to be able to explore humanoid hand possibilities on a real robot, and Reachy2 is the perfect candidate for that !

![Reachy](assets/Reachy.jpg)

Amazing Hand is a low-cost (<200€) and open-source 4 fingers hand, driven by parallel mechanism.

[AmazingHand_Overview"](/docs/AmazingHand_Overview.pdf)

![Hand Overview](assets/Hand_Overview.jpg)

Small Feetech SCS0009 servos are used to move each finger in flexion / extension, & Abduction / Adduction

![Finger Overview](assets/Finger_Overview.jpg)


2 ways of control are available :
- Use a Serial bus driver (waveshare i.e.) + Python script
- Use an arduino + feetech TTL Linker
Detailed explaination are available for both and Basic demo software is available also.
Up to you !


## Table of contents

- [Build Resources](#build-resources)
    - [BOM (Bill Of Materials)](#bom-bill-of-materials)
    - [STL Files and Onshape document](#stl-files-and-onshape-document)
    - [Assembly Guide](#assembly-guide)
    - [Run basic Demo](#Run-basic-Demo)
- [Disclaimer](#disclaimer)
- [Project Updates & Community](#project-updates--community)
    - [Updates history](#updates-history)
    - [Project posts](#project-posts)
    - [To Do List](#to-do-list)
    - [FAQ](#faq)
    - [Contact](#contact)
    - [Thank you](#thank-you)


# Build Resources
## BOM (Bill Of Materials)
List of all needed components is available here:  
[AmazingHand BOM](https://docs.google.com/spreadsheets/d/1QH2ePseqXjAhkWdS9oBYAcHPrxaxkSRCgM_kOK0m52E/edit?gid=1269903342#gid=1269903342)  
![BOM](assets/BOM.jpg)

Detailed for custom 3D printed parts are here : 
[3Dprinted parts](https://docs.google.com/spreadsheets/d/1QH2ePseqXjAhkWdS9oBYAcHPrxaxkSRCgM_kOK0m52E/edit?gid=2050623549#gid=2050623549)
![3Dparts](assets/3Dparts.jpg)


+ Control choice (2 options detailed previously)

## STL Files and Onshape document
STL and Steps files can be found [here](https://github.com/pollen-robotics/AmazingHand/tree/main/cad)  

Everyone can access the Onshape document too:   
[Link Onshape](To be Updated)  
![AmazingHand onshape picture](To be Updated)  

Note that predefined position are available in "named position" tooling, with corresponding servos angles

## Assembly Guide
Here is guide to explain how to print all the needed custom parts :
[=> 3D Printing Guide](/docs/AmazingHand_3DprintingTips.pdf)
![3DPrint_example](/assets/3DPrint.jpg) 

And then how to use them to build the Amazing Hand in combination with standards components in the BOM.  
[=> Assembly Guide](/docs/AmazingHand_Assembly.pdf)  
![Assembly_example](/assets/Assembly.jpg)  

## Run basic Demo

Basic Demo which is available with both Python & Arduino.
[Demo](assets/AmazingHand_BasicDemo.mov)

# Disclaimer

This design has not yet bene tested for long and complex prehensive tasks. Before to be able to grasp objetcs safely (that means without damaging servos or mehanical parts), kind of smart software need to be build.
SCS0009 servos have smart capaibilities as:
- Torque enable / disable
- Torque feedback
- Current position sensor
- Heat temperature feedback
- ...


# Project Updates & Community
## Updates history
[Updates history](/docs/changelog.md)  

## To Do List
- Test with prehensive tasks 
      => Add smarter behaviour for closing hand, based on avalable motors feedbacks
- Study possibility to use STS3032 feetedch motors instead of SCS0009
      => Stronger for quite the same volume, but servo horn is different


## FAQ
WIP

## Contact
[Contact me or Pollen Robotics](/docs/contact.md)

## Thank you
Huge thanks to those who have contributed to this project so far:
- [Steve N'Guyen](https://fr.linkedin.com/in/stevenguyen) for beta testing and very cool hand tracking demo
- [Pierre Rouanet](https://github.com/pierre-rouanet) for Feetech motors integration in pypot  
- [Augustin Crampette](https://fr.linkedin.com/in/augustin-crampette) & [Matthieu Lapeyre](https://www.linkedin.com/in/matthieulapeyre/) for open discussions and mechanical advices
