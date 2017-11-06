
from director import mainwindowapp
from director import robotsystem
from director import applogic as app
from director import consoleapp
from director import transformUtils
from director import mainwindowapp
from director import depthscanner
from director import ioUtils
from director import visualization as vis
import numpy as np
from director import objectmodel as om
from director import vtkAll as vtk
 
class CameraPoses(object):

    def __init__(self, posegraphFile=None):
        self.posegraphFile = posegraphFile

        if self.posegraphFile is not None:
            self.loadCameraPoses(posegraphFile)


    def loadCameraPoses(self, posegraphFile):
        data = np.loadtxt(posegraphFile)
        self.poseTimes = np.array(data[:,0]*1e6, dtype=int)
        self.poses = []
        for pose in data[:,1:]:
            pos = pose[:3]
            quat = pose[6], pose[3], pose[4], pose[5] # quat data from file is ordered as x, y, z, w
            self.poses.append((pos, quat))

    def getCameraPoseAtUTime(self, utime):
        idx = np.searchsorted(self.poseTimes, utime, side='left')
        if idx == len(self.poseTimes):
            idx = len(self.poseTimes) - 1

        (pos, quat) = self.poses[idx]
        return transformUtils.transformFromPose(pos, quat)

def setCameraTransform(camera, transform):
    '''Set camera transform so that view direction is +Z and view up is -Y'''
    origin = np.array(transform.GetPosition())
    axes = transformUtils.getAxesFromTransform(transform)
    camera.SetPosition(origin)
    camera.SetFocalPoint(origin+axes[2])
    camera.SetViewUp(-axes[1])

def setCameraInstrinsicsAsus(camera):
    principalX = 320.0
    principalY = 240.0
    focalLength = 528.0
    setCameraIntrinsics(camera, principalX, principalY, focalLength)

def setCameraIntrinsics(camera, principalX, principalY, focalLength):
    '''Note, call this function after setting the view dimensions'''
    imageWidth = 480
    imageHeight = 640

    wcx = -2*(principalX - float(imageWidth)/2) / imageWidth
    wcy =  2*(principalY - float(imageHeight)/2) / imageHeight
    viewAngle = focalLengthToViewAngle(focalLength, imageHeight)

    camera.SetWindowCenter(wcx, wcy)
    camera.SetViewAngle(viewAngle)

def focalLengthToViewAngle(focalLength, imageHeight):
    '''Returns a view angle in degrees that can be set on a vtkCamera'''
    return np.degrees(2.0 * np.arctan2(imageHeight/2.0, focalLength))

def encode_normal_rgb(view, height, width, pickType='cells', tolerance=0.05):
  #picktype is one of ('points', 'cells', 'render')
  image = np.zeros((height,width,3))
  for i in range(height):
    print i
    for j in range(width):
      pickPointFields = vis.pickPoint(
      [i,j],
      view,
      pickType=pickType,
      tolerance=tolerance)
      normal = np.array(pickPointFields.pickedNormal)
      image[i,j,:] = normal
    # add some rgb conversion step, maybe png writer does that???
  return image

if __name__ == '__main__':
  #setup
  view_height = 640
  view_width = 480
  data_dir = sys.argv[1]
  num_im = int(sys.argv[2])
  mesh = sys.argv[3]

  mapper =vtk.vtkPolyDataMapper()
  actor =vtk.vtkActor()
  renderer =vtk.vtkRenderer()
  renWin =vtk.vtkRenderWindow()
  interactor = vtk.vtkRenderWindowInteractor()
  fileReader = vtk.vtkPLYReader()
  filter1= vtk.vtkWindowToImageFilter()
  imageWriter =vtk.vtkBMPWriter()
  scale =vtk.vtkImageShiftScale()
  fileReader.SetFileName(sys.argv[1]+"/"+sys.argv[3])
  renWin.SetSize(view_height,view_width)

  camera = vtk.vtkCamera()
  renderer.SetActiveCamera(camera);

  mapper.SetInputConnection(fileReader.GetOutputPort());
  actor.SetMapper(mapper);
  renderer.AddActor(actor);
  renWin.AddRenderer(renderer);
  interactor.SetRenderWindow(renWin);

  setCameraInstrinsicsAsus(camera)
  setCameraTransform(camera, vtk.vtkTransform())

  filter1.SetInput(renWin)
  filter1.SetMagnification(1)
  filter1.SetInputBufferTypeToZBuffer()     
 
  scale.SetOutputScalarTypeToUnsignedChar()
  scale.SetInputConnection(filter1.GetOutputPort())
  scale.SetShift(0);
  scale.SetScale(-255);

  poses = CameraPoses(data_dir+"/posegraph.posegraph")

  for i in range(1,num_im+1):
      print "rendering image "+str(i)
      utimeFile = open(data_dir+"/images/"+ str(i).zfill(10) + "_utime.txt", 'r')
      utime = int(utimeFile.read())     
      #update camera transform
      cameraToCameraStart = poses.getCameraPoseAtUTime(utime)
      t = cameraToCameraStart
      setCameraTransform(camera, t)
      renWin.Render()
      
      #figure out why this is necessary
      filter1= vtk.vtkWindowToImageFilter()
      scale =vtk.vtkImageShiftScale()
      filter1.SetInput(renWin)
      filter1.SetMagnification(1)
      filter1.SetInputBufferTypeToZBuffer()     
 
      scale.SetOutputScalarTypeToUnsignedChar()
      scale.SetInputConnection(filter1.GetOutputPort())
      scale.SetShift(0);
      scale.SetScale(-255);

      #filter1.Update()
      #scale.Update()

      #Write depth map as a.bmp image gotta be shifted over ,,look at vertex map to normal map
      imageWriter.SetFileName(data_dir+"/images/"+str(i).zfill(10)+"depth_ground_truth.bmp");
      imageWriter.SetInputConnection(scale.GetOutputPort());
      imageWriter.Write();
  
  renWin.Render();
  interactor.Start();