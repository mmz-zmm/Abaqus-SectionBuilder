#### Using Abaqus Script to Generate Airfoil Section and Interior Stucture

##### Oct 18, 2016, NUAA

- Code was write by Python 3.4 using Anaconda, but can also work under Python 2.7 interpreter.


- The **module *SectionBuilderSubroutine.pyc*** used in the main program ***AbaqusAirfoilSectionBuilder.py*** shall be placed in the Abaqus working path, for example,

  '[F:\User\Documents\Work\Abaqus]()', in order to work properly. The path can be set in the GUI mode by clicking *File->Set Work Directory* .

- In the main script **AbaqusAirfoilSectionBuilder.py**, replace the *input_file* parameter with the full path of your input file, usually the airfoil data file.

- The  *input_file* (airfoil data) should be a close curve points set.

- The control point D which be used to decide the position of the C-beam, shalled by modifed for airfoil with high curvature.

- For more information, contact the author at  [zhaomzhao@nuaa.edu.cn](mailto://zhaomzhao@nuaa.edu.cn)

  ​

  ​