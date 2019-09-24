set(FIND_MCCL_PATHS
 ${CMAKE_CURRENT_SOURCE_DIR})
 
find_path(MCCL_INCLUDE_DIR MCCL_Fun.h
		PATH_SUFFIXES include 
		PATHS ${FIND_MCCL_PATHS})
		message(${FIND_MCCL_PATHS})
		message("-----------------------")
#find_library(wdapi1230_LIBRARY             NAMES wdapi1230.dll            PATH_SUFFIXES lib            PATHS ${FIND_BASICSTATICLIB_PATHS}            )    
find_library(MCCL_LIBRARY_lib
             NAMES MCCLPCI_IMP3
			 PATH_SUFFIXES LIBX32
			 PATHS ${FIND_MCCL_PATHS}
					)         
find_library(wdapi1230dll_LIB 
			NAMES wdapi1230
			PATH_SUFFIXES LIBX32
			PATHS ${FIND_MCCL_PATHS}
			)
		