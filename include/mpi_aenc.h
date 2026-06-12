/* Compatibility shim: this SDK (HI_SDK20190315 / 3516ev200) merges the audio
   MPI into a single mpi_audio.h. Kept so existing #include "mpi_aenc.h" works. */
#ifndef __MPI_AENC_SHIM_H__
#define __MPI_AENC_SHIM_H__
#include "mpi_audio.h"
#endif
