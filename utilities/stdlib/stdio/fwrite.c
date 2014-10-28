
/*HEADER**********************************************************************
*
* Copyright 2013 Freescale Semiconductor, Inc.
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale MQX RTOS License
* distributed with this Material.
* See the MQX_RTOS_LICENSE file distributed for more details.
*
* Brief License Summary:
* This software is provided in source form for you to use free of charge,
* but it is not open source software. You are allowed to use this software
* but you cannot redistribute it or derivative works of it in source form.
* The software may be used only in connection with a product containing
* a Freescale microprocessor, microcontroller, or digital signal processor.
* See license agreement file for full license terms including other restrictions.
*****************************************************************************
*
* Comments:
*
*   Contains the function fwrite.
*
*
*END************************************************************************/

#include <stdio.h>
#include "buf_prv.h"
/*!
 * \brief This function write to the stream.
 *
 * \param[in] ptr       Point to array  where this function reads elements.
 * \param[in] size      Size of element to be write.
 * \param[in] nmemb     Number of elements to be write.
 * \param[in] stream    Stream where function write
 *
 * \return The number of elements successfully write, which may be less than nmemb only if a write error encountered.
 */
size_t fwrite(const void *ptr, size_t size, size_t nmemb, FILE  *stream)
{
    int result;
    size_t count = 0;

    if ((NULL == ptr) || (NULL == stream) || (0 == size) || (0 == nmemb))
    {
        return 0; //error: bad input parameter
    }

    for (; 0 < nmemb; nmemb--)
    {
        result = _buf_write((const unsigned char*)ptr, size, stream);
        ptr = (unsigned char*)ptr + size;
        if (result != size)
        {
            /* Mark error */
            stream->_MODE |= _MODE_ERR;
            break;
        }
        count++;
    }

    return count;

}
