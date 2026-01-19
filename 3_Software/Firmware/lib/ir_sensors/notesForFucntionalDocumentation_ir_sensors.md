<!-- Note, it is intentionnal that the first heading starts as a "level2"/## and not as level1/# and should be kept like this!! -->
prompt:

Now on to the next .md-file, we'll now do:  IR

CORRECT AS NEEDED!!!!!!!!!!!!!!!!!!!!!!!!!!!!
The idea behind is to have the hovercraft in an autonomous driving mode, triggered when hit in the web-ui. Then craft then perfoms its automated loop-mission, until one reliability-flags fail (such as "timeout, no new line found) or the "start-mission" button is hit again. It then returns to the normal "manual/app" piloting mode automatically.

....


The goal is now to start documenting the files, for that I prepared for each key-functionnality a .md file. The goal is to produce a functional documentation, short enough so people will still read it and find key info, but deep enough so it can be used to understand the code.
For each I want: 
- A short section (1-2 sentences) explaining the main idea, what this does
- A short section (1-2 sentences) where the file is linked in, where are objects of this class generated, how is it called, ...

- functionnalities implemented: (not directly/strictly the methods(), but rather from a more global view, for ex: "complementary filter, to calculate ..."


- Methods, a short overview explaining the methods created for the class. 
- Parameters it uses (internal/ctor, ... and extern - reference to hovercraft_varibales, ...)

=> make sure to adapt the length of the file fitting to the class/file, if not much text is needed, keep it short!!