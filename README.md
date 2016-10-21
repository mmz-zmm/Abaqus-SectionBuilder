#### Using Abaqus Script to Generate Airfoil Section and Interior Stucture

##### Oct, eighteen, 2016, by Zhao Mengmeng, NUAA

- The **module *SectionBuilderSubroutine.py*** used in the main program ***AbaqusAirfoilSectionBuilder.py*** shall be placed in the Abaqus working path, for example,

  '[F:\User\Documents\Work\Abaqus]()', in order to work properly. The path can be set in the GUI mode by clicking *File->Set Work Directory*

- In the main script **AbaqusAirfoilSectionBuilder.py**, replace the *input_file* parameter with the full path of your input file, usually the airfoil data file.

- The  *input_file* (airfoil data) should be a close curve points set.

- The control point D which be used to decide the position of the C-beam, shalled by modifed for airfoil with high curvature.

- For more information, contact the author at  [zhaomzhao@nuaa.edu.cn](mailto://zhaomzhao@nuaa.edu.cn)

  ​

  ​