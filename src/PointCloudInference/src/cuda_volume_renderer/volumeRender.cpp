/*
 * Based on the CUDA 7.5 Volume Rendering example, with below copyright.
 *
 * Copyright 1993-2015 NVIDIA Corporation.  All rights reserved.
 *
 * Please refer to the NVIDIA end user license agreement (EULA) associated
 * with this source code for terms and conditions that govern your use of
 * this software. Any use, reproduction, disclosure, or distribution of
 * this software and related documentation outside the terms of the EULA
 * is strictly prohibited.
 *
 * Modified by gizatt (with the Robot Locomotion Group), 2017.
 */

/*
    Simple volume rendering functionality for visualizing
    3D volumes quickly with a GPU.

    This sample loads a 3D volume from disk and displays it using
    ray marching and 3D textures. Not particularly efficient,
    but by the magic of GPU, it's fast enough.
*/

// OpenGL Graphics includes
#include <GL/glew.h>
#if defined(__APPLE__) || defined(MACOSX)
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#include <GLUT/glut.h>
#ifndef glutCloseFunc
#define glutCloseFunc glutWMCloseFunc
#endif
#else
#include <GL/freeglut.h>
#endif

// CUDA Runtime, Interop, and includes
#include <cuda_gl_interop.h>
#include <cuda_runtime.h>
#include <driver_functions.h>
#include <vector_functions.h>
#include <vector_types.h>

// CUDA utilities
#include "helper_cuda.h"
#include "helper_cuda_gl.h"

// Helper functions
#include "helper_cuda.h"
#include "helper_functions.h"
#include "helper_timer.h"

#include "../argagg.hpp"
#include "../voxel_distance_field.h"

using namespace std;

typedef unsigned int uint;
typedef unsigned char uchar;

#define MAX_EPSILON_ERROR 5.00f
#define THRESHOLD 0.30f

const char *sSDKsample = "CUDA 3D Volume Render";

const char *volumeFilename;
cudaExtent volumeSize;
typedef unsigned char VolumeType;

uint width = 1280, height = 720;
dim3 blockSize(16, 16);
dim3 gridSize;

float3 viewRotation;
float3 viewTranslation = make_float3(0.0, 0.0, -4.0f);
float invViewMatrix[12];

float density = 0.05f;
float brightness = 1.0f;
float transferOffset = 0.0f;
float transferScale = 1.0f;
bool linearFiltering = true;

GLuint pbo = 0;  // OpenGL pixel buffer object
GLuint tex = 0;  // OpenGL texture object
struct cudaGraphicsResource
    *cuda_pbo_resource;  // CUDA Graphics Resource (to transfer PBO)

StopWatchInterface *timer = 0;

// Auto-Verification Code
const int frameCheckNumber = 2;
int fpsCount = 0;  // FPS count for averaging
int fpsLimit = 1;  // FPS limit for sampling
int g_Index = 0;
unsigned int frameCount = 0;

int *pArgc;
char **pArgv;

#ifndef MAX
#define MAX(a, b) ((a > b) ? a : b)
#endif

extern "C" void setTextureFilterMode(bool bLinearFilter);
extern "C" void initCuda(void *h_volume, cudaExtent volumeSize);
extern "C" void freeCudaBuffers();
extern "C" void render_kernel(dim3 gridSize, dim3 blockSize, uint *d_output,
                              uint imageW, uint imageH, float density,
                              float brightness, float transferOffset,
                              float transferScale);
extern "C" void copyInvViewMatrix(float *invViewMatrix, size_t sizeofMatrix);

void initPixelBuffer();

void computeFPS() {
  frameCount++;
  fpsCount++;

  if (fpsCount == fpsLimit) {
    char fps[256];
    float ifps = 1.f / (sdkGetAverageTimerValue(&timer) / 1000.f);
    sprintf(fps, "Volume Render: %3.1f fps", ifps);

    glutSetWindowTitle(fps);
    fpsCount = 0;

    fpsLimit = (int)MAX(1.f, ifps);
    sdkResetTimer(&timer);
  }
}

// render image using CUDA
void render() {
  copyInvViewMatrix(invViewMatrix, sizeof(float4) * 3);

  // map PBO to get CUDA device pointer
  uint *d_output;
  // map PBO to get CUDA device pointer
  checkCudaErrors(cudaGraphicsMapResources(1, &cuda_pbo_resource, 0));
  size_t num_bytes;
  checkCudaErrors(cudaGraphicsResourceGetMappedPointer(
      (void **)&d_output, &num_bytes, cuda_pbo_resource));
  // printf("CUDA mapped PBO: May access %ld bytes\n", num_bytes);

  // clear image
  checkCudaErrors(cudaMemset(d_output, 0, width * height * 4));

  // call CUDA kernel, writing results to PBO
  render_kernel(gridSize, blockSize, d_output, width, height, density,
                brightness, transferOffset, transferScale);

  getLastCudaError("kernel failed");

  checkCudaErrors(cudaGraphicsUnmapResources(1, &cuda_pbo_resource, 0));
}

// display results using OpenGL (called by GLUT)
void display() {
  sdkStartTimer(&timer);

  // use OpenGL to build view matrix
  GLfloat modelView[16];
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glRotatef(-viewRotation.x, 1.0, 0.0, 0.0);
  glRotatef(-viewRotation.y, 0.0, 1.0, 0.0);
  glTranslatef(-viewTranslation.x, -viewTranslation.y, -viewTranslation.z);
  glGetFloatv(GL_MODELVIEW_MATRIX, modelView);
  glPopMatrix();

  invViewMatrix[0] = modelView[0];
  invViewMatrix[1] = modelView[4];
  invViewMatrix[2] = modelView[8];
  invViewMatrix[3] = modelView[12];
  invViewMatrix[4] = modelView[1];
  invViewMatrix[5] = modelView[5];
  invViewMatrix[6] = modelView[9];
  invViewMatrix[7] = modelView[13];
  invViewMatrix[8] = modelView[2];
  invViewMatrix[9] = modelView[6];
  invViewMatrix[10] = modelView[10];
  invViewMatrix[11] = modelView[14];

  render();

  // display results
  glClear(GL_COLOR_BUFFER_BIT);

  // draw image from PBO
  glDisable(GL_DEPTH_TEST);

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
#if 0
    // draw using glDrawPixels (slower)
    glRasterPos2i(0, 0);
    glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, pbo);
    glDrawPixels(width, height, GL_RGBA, GL_UNSIGNED_BYTE, 0);
    glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);
#else
  // draw using texture

  // copy from pbo to texture
  glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, pbo);
  glBindTexture(GL_TEXTURE_2D, tex);
  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGBA,
                  GL_UNSIGNED_BYTE, 0);
  glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);

  // draw textured quad
  glEnable(GL_TEXTURE_2D);
  glBegin(GL_QUADS);
  glTexCoord2f(0, 0);
  glVertex2f(0, 0);
  glTexCoord2f(1, 0);
  glVertex2f(1, 0);
  glTexCoord2f(1, 1);
  glVertex2f(1, 1);
  glTexCoord2f(0, 1);
  glVertex2f(0, 1);
  glEnd();

  glDisable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, 0);
#endif

  glutSwapBuffers();
  glutReportErrors();

  sdkStopTimer(&timer);

  computeFPS();
}

void idle() { glutPostRedisplay(); }

void keyboard(unsigned char key, int x, int y) {
  switch (key) {
    case 27:
#if defined(__APPLE__) || defined(MACOSX)
      exit(EXIT_SUCCESS);
#else
      glutDestroyWindow(glutGetWindow());
      return;
#endif
      break;

    case 'f':
      linearFiltering = !linearFiltering;
      setTextureFilterMode(linearFiltering);
      break;

    case '+':
      density += 0.01f;
      break;

    case '-':
      density -= 0.01f;
      break;

    case ']':
      brightness += 0.1f;
      break;

    case '[':
      brightness -= 0.1f;
      break;

    case ';':
      transferOffset += 0.01f;
      break;

    case '\'':
      transferOffset -= 0.01f;
      break;

    case '.':
      transferScale += 0.01f;
      break;

    case ',':
      transferScale -= 0.01f;
      break;

    default:
      break;
  }

  printf(
      "density = %.2f, brightness = %.2f, transferOffset = %.2f, transferScale "
      "= %.2f\n",
      density, brightness, transferOffset, transferScale);
  glutPostRedisplay();
}

int ox, oy;
int buttonState = 0;

void mouse(int button, int state, int x, int y) {
  if (state == GLUT_DOWN) {
    buttonState |= 1 << button;
  } else if (state == GLUT_UP) {
    buttonState = 0;
  }

  ox = x;
  oy = y;
  glutPostRedisplay();
}

void motion(int x, int y) {
  float dx, dy;
  dx = (float)(x - ox);
  dy = (float)(y - oy);

  if (buttonState == 4) {
    // right = zoom
    viewTranslation.z += dy / 100.0f;
  } else if (buttonState == 2) {
    // middle = translate
    viewTranslation.x += dx / 100.0f;
    viewTranslation.y -= dy / 100.0f;
  } else if (buttonState == 1) {
    // left = rotate
    viewRotation.x += dy / 5.0f;
    viewRotation.y += dx / 5.0f;
  }

  ox = x;
  oy = y;
  glutPostRedisplay();
}

int iDivUp(int a, int b) { return (a % b != 0) ? (a / b + 1) : (a / b); }

void reshape(int w, int h) {
  width = w;
  height = h;
  initPixelBuffer();

  // calculate new grid size
  gridSize = dim3(iDivUp(width, blockSize.x), iDivUp(height, blockSize.y));

  glViewport(0, 0, w, h);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);
}

void cleanup() {
  sdkDeleteTimer(&timer);

  freeCudaBuffers();

  if (pbo) {
    cudaGraphicsUnregisterResource(cuda_pbo_resource);
    glDeleteBuffersARB(1, &pbo);
    glDeleteTextures(1, &tex);
  }
  // cudaDeviceReset causes the driver to clean up all state. While
  // not mandatory in normal operation, it is good practice.  It is also
  // needed to ensure correct operation when the application is being
  // profiled. Calling cudaDeviceReset causes all profile data to be
  // flushed before the application exits
  cudaDeviceReset();
}

void initGL(int *argc, char **argv) {
  // initialize GLUT callback functions
  glutInit(argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowSize(width, height);
  glutCreateWindow("CUDA volume rendering");

  glewInit();

  if (!glewIsSupported("GL_VERSION_2_0 GL_ARB_pixel_buffer_object")) {
    printf("Required OpenGL extensions missing.");
    exit(EXIT_SUCCESS);
  }
}

void initPixelBuffer() {
  if (pbo) {
    // unregister this buffer object from CUDA C
    checkCudaErrors(cudaGraphicsUnregisterResource(cuda_pbo_resource));

    // delete old buffer
    glDeleteBuffersARB(1, &pbo);
    glDeleteTextures(1, &tex);
  }

  // create pixel buffer object for display
  glGenBuffersARB(1, &pbo);
  glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, pbo);
  glBufferDataARB(GL_PIXEL_UNPACK_BUFFER_ARB,
                  width * height * sizeof(GLubyte) * 4, 0, GL_STREAM_DRAW_ARB);
  glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);

  // register this buffer object with CUDA
  checkCudaErrors(cudaGraphicsGLRegisterBuffer(
      &cuda_pbo_resource, pbo, cudaGraphicsMapFlagsWriteDiscard));

  // create texture for display
  glGenTextures(1, &tex);
  glBindTexture(GL_TEXTURE_2D, tex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA,
               GL_UNSIGNED_BYTE, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glBindTexture(GL_TEXTURE_2D, 0);
}

// Load raw data from disk
void *loadRawFile(char *filename, size_t size) {
  FILE *fp = fopen(filename, "rb");

  if (!fp) {
    fprintf(stderr, "Error opening file '%s'\n", filename);
    return 0;
  }

  void *data = malloc(size);
  size_t read = fread(data, 1, size, fp);
  fclose(fp);

#if defined(_MSC_VER_)
  printf("Read '%s', %Iu bytes\n", filename, read);
#else
  printf("Read '%s', %zu bytes\n", filename, read);
#endif

  return data;
}

// General initialization call for CUDA Device
int chooseCudaDevice(int argc, const char **argv, bool bUseOpenGL) {
  int result = 0;

  if (bUseOpenGL) {
    result = findCudaGLDevice(argc, argv);
  } else {
    result = findCudaDevice(argc, argv);
  }

  return result;
}

void runSingleTest(const char *ref_file, const char *exec_path) {
  bool bTestResult = true;

  uint *d_output;
  checkCudaErrors(
      cudaMalloc((void **)&d_output, width * height * sizeof(uint)));
  checkCudaErrors(cudaMemset(d_output, 0, width * height * sizeof(uint)));

  float modelView[16] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                         0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 4.0f, 1.0f};

  invViewMatrix[0] = modelView[0];
  invViewMatrix[1] = modelView[4];
  invViewMatrix[2] = modelView[8];
  invViewMatrix[3] = modelView[12];
  invViewMatrix[4] = modelView[1];
  invViewMatrix[5] = modelView[5];
  invViewMatrix[6] = modelView[9];
  invViewMatrix[7] = modelView[13];
  invViewMatrix[8] = modelView[2];
  invViewMatrix[9] = modelView[6];
  invViewMatrix[10] = modelView[10];
  invViewMatrix[11] = modelView[14];

  // call CUDA kernel, writing results to PBO
  copyInvViewMatrix(invViewMatrix, sizeof(float4) * 3);

  // Start timer 0 and process n loops on the GPU
  int nIter = 10;

  for (int i = -1; i < nIter; i++) {
    if (i == 0) {
      cudaDeviceSynchronize();
      sdkStartTimer(&timer);
    }

    render_kernel(gridSize, blockSize, d_output, width, height, density,
                  brightness, transferOffset, transferScale);
  }

  cudaDeviceSynchronize();
  sdkStopTimer(&timer);
  // Get elapsed time and throughput, then log to sample and master logs
  double dAvgTime = sdkGetTimerValue(&timer) / (nIter * 1000.0);
  printf(
      "volumeRender, Throughput = %.4f MTexels/s, Time = %.5f s, Size = %u "
      "Texels, NumDevsUsed = %u, Workgroup = %u\n",
      (1.0e-6 * width * height) / dAvgTime, dAvgTime, (width * height), 1,
      blockSize.x * blockSize.y);

  getLastCudaError("Error: render_kernel() execution FAILED");
  checkCudaErrors(cudaDeviceSynchronize());

  unsigned char *h_output = (unsigned char *)malloc(width * height * 4);
  checkCudaErrors(cudaMemcpy(h_output, d_output, width * height * 4,
                             cudaMemcpyDeviceToHost));

  sdkSavePPM4ub("volume.ppm", h_output, width, height);
  bTestResult =
      sdkComparePPM("volume.ppm", sdkFindFilePath(ref_file, exec_path),
                    MAX_EPSILON_ERROR, THRESHOLD, true);

  cudaFree(d_output);
  free(h_output);
  cleanup();

  exit(bTestResult ? EXIT_SUCCESS : EXIT_FAILURE);
}

////////////////////////////////////////////////////////////////////////////////
// Program main
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
  pArgc = &argc;
  pArgv = argv;

  char *ref_file = NULL;

#if defined(__linux__)
  setenv("DISPLAY", ":0", 0);
#endif

  // start logs
  printf("%s Starting...\n\n", sSDKsample);

  argagg::parser argparser{
      {{"help", {"-h", "--help"}, "shows this help message", 0},
       {"file", {"-f", "--file"}, "Volume distance function file.", 1},
       {"output", {"-o", "--output"}, "Output file.", 1}}};

  argagg::parser_results args;
  try {
    args = argparser.parse(argc, argv);
  } catch (const std::exception &e) {
    cerr << e.what() << endl;
    return -1;
  }

  if (args["help"] || !args["file"]) {
    cerr << argparser;
    return 0;
  }

  string input_file = args["file"].as<string>();

  // First initialize OpenGL context, so we can properly set the GL for CUDA.
  // This is necessary in order to achieve optimal performance with
  // OpenGL/CUDA interop.
  initGL(&argc, argv);

  // use command-line specified CUDA device, otherwise use device with highest
  // Gflops/s
  chooseCudaDevice(argc, (const char **)argv, true);

  printf("Loading vdf...\n");
  VoxelDistanceField vdf(input_file);
  int w = vdf.GetSize()[0];
  int h = vdf.GetSize()[1];
  int d = vdf.GetSize()[2];
  volumeSize.width = w;
  volumeSize.height = h;
  volumeSize.depth = d;
  printf("Loaded size %d, %d, %d\n", w, h, d);

  // Load the actual volume of interest
  std::vector<VolumeType> h_volume(
      volumeSize.width * volumeSize.height * volumeSize.depth, 0);
  auto occupied_nodes = vdf.GetOccupiedNodes();
  for (const auto &node : occupied_nodes) {
    h_volume[node[0] * h * d + node[1] * d + node[2]] = 255;
    printf("Node %d, %d, %d occupied\n", node[0], node[1], node[2]);
  }
  initCuda(h_volume.data(), volumeSize);

  sdkCreateTimer(&timer);

  printf(
      "Press '+' and '-' to change density (0.01 increments)\n"
      "      ']' and '[' to change brightness\n"
      "      ';' and ''' to modify transfer function offset\n"
      "      '.' and ',' to modify transfer function scale\n\n");

  // calculate new grid size
  gridSize = dim3(iDivUp(width, blockSize.x), iDivUp(height, blockSize.y));

  if (ref_file) {
    runSingleTest(ref_file, argv[0]);
  } else {
    // This is the normal rendering path for VolumeRender
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);

    initPixelBuffer();

#if defined(__APPLE__) || defined(MACOSX)
    atexit(cleanup);
#else
    glutCloseFunc(cleanup);
#endif

    glutMainLoop();
  }
}
