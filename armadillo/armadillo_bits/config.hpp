// Copyright (C) 2010 NICTA and the authors listed below
// http://nicta.com.au
// 
// Authors:
// - Conrad Sanderson (conradsand at ieee dot org)
// 
// This file is part of the Armadillo C++ library.
// It is provided without any warranty of fitness
// for any purpose. You can redistribute this file
// and/or modify it under the terms of the GNU
// Lesser General Public License (LGPL) as published
// by the Free Software Foundation, either version 3
// of the License or (at your option) any later version.
// (see http://www.opensource.org/licenses for more info)



// #define ARMA_HAVE_STD_ISFINITE
// #define ARMA_HAVE_STD_ISINF
// #define ARMA_HAVE_STD_ISNAN
// #define ARMA_HAVE_STD_SNPRINTF

// #define ARMA_HAVE_LOG1P
// #define ARMA_HAVE_GETTIMEOFDAY

// #define ARMA_USE_ATLAS
// #define ARMA_USE_LAPACK
// #define ARMA_USE_BLAS
// #define ARMA_USE_BOOST
// #define ARMA_USE_BOOST_DATE

// #define ARMA_EXTRA_DEBUG
// #define ARMA_NO_DEBUG

#if defined(ARMA_USE_ATLAS)
  #if !defined(ARMA_ATLAS_INCLUDE_DIR)
    #define ARMA_ATLAS_INCLUDE_DIR
  #endif
#endif
