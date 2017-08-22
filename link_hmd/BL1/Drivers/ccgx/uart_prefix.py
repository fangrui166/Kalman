#!/usr/bin/env python2

# Written by wanjin_shi - 2017/01/18

import sys,struct,zlib,os,time
from optparse import OptionParser
#from intelhex import IntelHex

path = sys.path[0]

BL1_ver_file = path + '/../BL1/Drivers/fota/htc_version.h'
SYS_ver_file = path + '/../SYS/Inc/htc_version.h'

def named(tuple,names):
  return dict(zip(names.split(),tuple))
def consume(fmt,data,names):
  n = struct.calcsize(fmt)
  return named(struct.unpack(fmt,data[:n]),names),data[n:]
def cstring(string):
  return string.split('\0',1)[0]
def compute_crc(data):
  return 0xFFFFFFFF & -zlib.crc32(data) -1
def build(file,targets):
  data = ''
  for t,target in enumerate(targets):
    tdata = ''
    for bin in target:
      crc   = compute_crc(bin['data'])
      tdata = struct.pack('<16s16s16s16s16s2I16s24s',bin['projectname']+'\0',bin['subprojectname']+'\0',
                          bin['version']+'\0',bin['builddate']+'\0',bin['buildtime']+'\0',crc,len(bin['data']),bin['index']+'\0','\0')+bin['data']
      print 'swj'
      open(file,'wb').write(tdata)

def IsSubString(SubStrList,Str):
    '''''
    #>>>Str='F06925EMS91.txt'
    #>>>IsSubString(SubStrList,Str)#return True (or False)
    '''
    flag=True
    for substr in SubStrList:
        if not(substr in Str):
            flag=False
    return flag

def GetFileList(FindPath,FlagStr=[]):
    '''''
    #>>>FileList=GetFileList(FindPath,FlagStr) #
    '''
    import os
    FileList=[]
    FileNames=os.listdir(FindPath)
    if (len(FileNames)>0):
        for fn in FileNames:
            if (len(FlagStr)>0):
                if (IsSubString(FlagStr,fn)):
                    fullfilename=os.path.join(FindPath,fn)
                    print fullfilename
                    FileList.append(fullfilename)
    if (len(FileList)>0):
        FileList.sort()
    return FileList

def GetCCG4Version(CYACDFile):
    import os
    import re
    print CYACDFile
    with open(CYACDFile, 'r') as f:
        for line in f.readlines():
            if '626E' in line:
                break;
    print line
    match = re.findall(r'626E(\w{4})',line)
    if match:
        print match[0]
    return match[0]

def GetFwIndex(CYACDFile):
    import os
    import re
    print CYACDFile
    with open(CYACDFile, 'r') as f:
        for line in f.readlines():
            pattern1 = re.compile(r':000014')
            match1 = pattern1.match(line)
            pattern2 = re.compile(r':000100')
            match2 = pattern2.match(line)
            if match1:
                print "firmware1"
                return "FW1"
            if match2:
                print "firmware2"
                return "FW2"




if __name__=="__main__":
    usage = """
    %prog [-f|--files] infile.bin
    %prog {-p|--project} project_name:subproject_name [-p project:subproject ...] outfile.bin
    """
    parser = OptionParser(usage=usage)
    parser.add_option("-f", "--files", action="append", dest="binfile",
    help="add a prefix from given BINFILES", metavar="BINFILES")
    parser.add_option("-p", "--project", action="append", dest="projectname",
    help="project name: sub project name", metavar="NAME")
    (options, args) = parser.parse_args()

    filelist = GetFileList(path,'cyacd')
    if len(filelist)!=2:
        print "ccg4 file error, please put 2 cyacd file in the git"
        sys.exit(1)

    print options.binfile
    target = []

    if len(filelist) == 2:
      for bf in filelist:
        if not os.path.isfile(bf):
          print "Unreadable file '%s'." % bf
          sys.exit(1)
        try:
          projectname,subprojectname = options.projectname[0].split(':',1)
        except ValueError:
          print "projectname:subname '%s' invalid." % options.projectname[0]
          sys.exit(1)
        if subprojectname == 'bl1':
          print subprojectname
          with open(BL1_ver_file, 'r') as f:
            for line in f.readlines():
              if 'SW_BL_VER' in line:
                print line
                version = line[19:29]
                print version
        elif subprojectname == 'sys':
          print subprojectname
          with open(SYS_ver_file, 'r') as f:
            for line in f.readlines():
              if 'SW_SYS_VER' in line:
                version = line[20:30]
                print version
        elif subprojectname == 'ccg4':
            version = GetCCG4Version(bf)
            index1 = GetFwIndex(bf)
        else:
          version = '1.01.001.1'
        print bf
        cdate = time.strftime("%Y-%m-%d",time.localtime(os.path.getmtime(bf)))
        ctime = time.strftime("%H:%M:%S",time.localtime(os.path.getmtime(bf)))
        print cdate
        print ctime
        target.append({ 'binfile':bf , 'projectname':projectname, 'subprojectname':subprojectname,
            'builddate':cdate,'buildtime':ctime,'version':version,'data': open(bf,'rb').read(),'index':index1 })

        outfile = bf+".bin"
        build(outfile,[target])
    else:
        parser.print_help()
        sys.exit(1)
