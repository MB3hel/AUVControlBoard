#!/usr/bin/env python3

import glob
import os
import shutil


def dofixup(filepath):
    newfilepath = "{0}.new".format(filepath)
    with open(newfilepath, 'w', encoding='utf8') as newfile:
        with open(filepath, 'r', encoding='utf8') as oldfile:
            for line in oldfile:
                if line.lstrip().startswith("#include"):
                    first_quote = line.find("\"")
                    second_quote = line.find("\"", first_quote+1)
                    if first_quote != -1 and second_quote != -1:
                        line = line[:first_quote] + "<" + line[first_quote+1:]
                        line = line[:second_quote] + ">" + line[second_quote+1:]
                newfile.write(line)
    os.remove(filepath)
    shutil.move(newfilepath, filepath)


if __name__ == "__main__":
    script_path = os.path.dirname(__file__).replace("\\","/")
    src_dir = "{0}/src".format(script_path)
    inc_dir = "{0}/include".format(script_path)

    # Make include folder
    print("Making empty include directory")
    if os.path.exists(inc_dir):
        shutil.rmtree(inc_dir)
    os.mkdir(inc_dir)

    # Move headers to include folder
    print("Moving headers to include directory")
    for filepath in glob.glob("{0}/**/*.h".format(src_dir), recursive=True):
        filepath = filepath.replace("\\", "/")
        destpath = filepath.replace(src_dir, inc_dir)
        destdir = os.path.dirname(destpath)
        if not os.path.exists(destdir):
            os.makedirs(destdir)
        shutil.move(filepath, destpath)

    # Change all includes to be absolute
    print("Making includes absolute")
    for filepath in glob.glob("{0}/**/*.c".format(src_dir), recursive=True):
        dofixup(filepath)
    for filepath in glob.glob("{0}/**/*.h".format(inc_dir), recursive=True):
        dofixup(filepath)
