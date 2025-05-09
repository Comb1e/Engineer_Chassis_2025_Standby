/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        FileSink.h
 * Description:  Node for creating File sinks
 *
 * $Date:        30 July 2021
 * $Revision:    V1.10.0
 *
 * Target Processor: Cortex-M and Cortex-A cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2021 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */ 
#ifndef _FILESINK_H_
#define _FILESINK_H_

/* Write a list of samples to a file in text form */
template<typename IN, int inputSize>
class FileSink: public GenericSink<IN, inputSize>
{
public:
    FileSink(FIFOBase<IN> &src, std::string name):GenericSink<IN,inputSize>(src),output(name){};

    int prepareForRunning() override
    {
        if (this->willUnderflow()
           )
        {
           return(CG_SKIP_EXECUTION_ID_CODE); // Skip execution
        }

        return(0);
    };
    
    int run() override
    {
        IN *b=this->getReadBuffer();

        for(int i=0;i<inputSize;i++)
        {
            output << b[i] << std::endl;
        }

        return(0);
    };

    ofstream output;
};

#endif