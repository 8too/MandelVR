#include <stdio.h>

#include "MainApplication.h"

int main(int argc, char* argv[]) {
  MainApplication* pMainApplication = new MainApplication(argc, argv);

  if (!pMainApplication->BInit()) {
    pMainApplication->Shutdown();
    return 1;
  }

  pMainApplication->RunMainLoop();

  pMainApplication->Shutdown();

  delete pMainApplication;

  return 0;
}
