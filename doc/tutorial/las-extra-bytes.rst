.. _reading:

===============================================================================
Reading and writing VLR's and extrabytes from LAS files with PDAL
===============================================================================

:Author: Kristian Evers
:Contact: kristianevers@gmail.com
:Date: 10/08/2016



.. contents:: Contents
   :depth: 3
   :backlinks: none

This tutorial will show how to work with the Variable Length Records (VLR)
and extra bytes in the LAS format. The examples uses the `RIEGL extrabytes`_
extension to the LAS format. The concepts are applicable to other usages of
VLR's and extrabytes.

.. _`RIEGL extrabytes`: http://www.riegl.com/uploads/tx_pxpriegldownloads/Whitepaper_-_LAS_extrabytes_implementation_in_Riegl_software_01.pdf

Introduction
------------------------------------------------------------------------------
The LAS format can be extended by user-defined data in two ways. Either with
Variable Length Records (VLR) or extra bytes added to the end of a Point Data
Record. VLR's can be used to describe various metadata related to a file and
the extrabytes offer a the user a way of adding custom dimensions to a Point
Data Record.

Variable Length Records
------------------------------------------------------------------------------

=========================== =============== =========== ========
Item                        Format          Size        Required
=========================== =============== =========== ========
(1.1) Reserved              unsigned short  2 bytes
User ID                     char[16]        16 bytes    *
Record ID                   unsigned short  2 bytes     *
Record Length After Header  unsigned short  2 bytes     *
Description                 char[32]        32 bytes
=========================== =============== =========== ========

Extrabytes
------------------------------------------------------------------------------


