// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file DDS_Data.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace {
char dummy;
}  // namespace
#endif  // _WIN32

#include "DDS_Data.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

PoseDDS::PoseDDS()
{
    // m_x com.eprosima.idl.parser.typecode.PrimitiveTypeCode@1f36e637
    m_x = 0.0;
    // m_y com.eprosima.idl.parser.typecode.PrimitiveTypeCode@578486a3
    m_y = 0.0;
    // m_z com.eprosima.idl.parser.typecode.PrimitiveTypeCode@551aa95a
    m_z = 0.0;
    // m_theta com.eprosima.idl.parser.typecode.PrimitiveTypeCode@35d176f7
    m_theta = 0.0;

}

PoseDDS::~PoseDDS()
{




}

PoseDDS::PoseDDS(
        const PoseDDS& x)
{
    m_x = x.m_x;
    m_y = x.m_y;
    m_z = x.m_z;
    m_theta = x.m_theta;
}

PoseDDS::PoseDDS(
        PoseDDS&& x) noexcept 
{
    m_x = x.m_x;
    m_y = x.m_y;
    m_z = x.m_z;
    m_theta = x.m_theta;
}

PoseDDS& PoseDDS::operator =(
        const PoseDDS& x)
{

    m_x = x.m_x;
    m_y = x.m_y;
    m_z = x.m_z;
    m_theta = x.m_theta;

    return *this;
}

PoseDDS& PoseDDS::operator =(
        PoseDDS&& x) noexcept
{

    m_x = x.m_x;
    m_y = x.m_y;
    m_z = x.m_z;
    m_theta = x.m_theta;

    return *this;
}

bool PoseDDS::operator ==(
        const PoseDDS& x) const
{

    return (m_x == x.m_x && m_y == x.m_y && m_z == x.m_z && m_theta == x.m_theta);
}

bool PoseDDS::operator !=(
        const PoseDDS& x) const
{
    return !(*this == x);
}

size_t PoseDDS::getMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 16 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8); // 128 bits, but aligned as 64

    current_alignment += 16 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8); // 128 bits, but aligned as 64

    current_alignment += 16 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8); // 128 bits, but aligned as 64

    current_alignment += 16 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8); // 128 bits, but aligned as 64


    return current_alignment - initial_alignment;
}

size_t PoseDDS::getCdrSerializedSize(
        const PoseDDS& data,
        size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 16 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8); // 128 bits, but aligned as 64

    current_alignment += 16 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8); // 128 bits, but aligned as 64

    current_alignment += 16 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8); // 128 bits, but aligned as 64

    current_alignment += 16 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8); // 128 bits, but aligned as 64


    return current_alignment - initial_alignment;
}

void PoseDDS::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{

    scdr << m_x;
    scdr << m_y;
    scdr << m_z;
    scdr << m_theta;

}

void PoseDDS::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{

    dcdr >> m_x;
    dcdr >> m_y;
    dcdr >> m_z;
    dcdr >> m_theta;
}

/*!
 * @brief This function sets a value in member x
 * @param _x New value for member x
 */
void PoseDDS::x(
        long double _x)
{
    m_x = _x;
}

/*!
 * @brief This function returns the value of member x
 * @return Value of member x
 */
long double PoseDDS::x() const
{
    return m_x;
}

/*!
 * @brief This function returns a reference to member x
 * @return Reference to member x
 */
long double& PoseDDS::x()
{
    return m_x;
}

/*!
 * @brief This function sets a value in member y
 * @param _y New value for member y
 */
void PoseDDS::y(
        long double _y)
{
    m_y = _y;
}

/*!
 * @brief This function returns the value of member y
 * @return Value of member y
 */
long double PoseDDS::y() const
{
    return m_y;
}

/*!
 * @brief This function returns a reference to member y
 * @return Reference to member y
 */
long double& PoseDDS::y()
{
    return m_y;
}

/*!
 * @brief This function sets a value in member z
 * @param _z New value for member z
 */
void PoseDDS::z(
        long double _z)
{
    m_z = _z;
}

/*!
 * @brief This function returns the value of member z
 * @return Value of member z
 */
long double PoseDDS::z() const
{
    return m_z;
}

/*!
 * @brief This function returns a reference to member z
 * @return Reference to member z
 */
long double& PoseDDS::z()
{
    return m_z;
}

/*!
 * @brief This function sets a value in member theta
 * @param _theta New value for member theta
 */
void PoseDDS::theta(
        long double _theta)
{
    m_theta = _theta;
}

/*!
 * @brief This function returns the value of member theta
 * @return Value of member theta
 */
long double PoseDDS::theta() const
{
    return m_theta;
}

/*!
 * @brief This function returns a reference to member theta
 * @return Reference to member theta
 */
long double& PoseDDS::theta()
{
    return m_theta;
}


size_t PoseDDS::getKeyMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t current_align = current_alignment;







    return current_align;
}

bool PoseDDS::isKeyDefined()
{
    return false;
}

void PoseDDS::serializeKey(
        eprosima::fastcdr::Cdr& scdr) const
{
    (void) scdr;
        
}

PointCloudDDS::PointCloudDDS()
{
    // m_seq com.eprosima.idl.parser.typecode.PrimitiveTypeCode@13eb8acf
    m_seq = 0;
    // m_points com.eprosima.idl.parser.typecode.SequenceTypeCode@51c8530f


}

PointCloudDDS::~PointCloudDDS()
{


}

PointCloudDDS::PointCloudDDS(
        const PointCloudDDS& x)
{
    m_seq = x.m_seq;
    m_points = x.m_points;
}

PointCloudDDS::PointCloudDDS(
        PointCloudDDS&& x) noexcept 
{
    m_seq = x.m_seq;
    m_points = std::move(x.m_points);
}

PointCloudDDS& PointCloudDDS::operator =(
        const PointCloudDDS& x)
{

    m_seq = x.m_seq;
    m_points = x.m_points;

    return *this;
}

PointCloudDDS& PointCloudDDS::operator =(
        PointCloudDDS&& x) noexcept
{

    m_seq = x.m_seq;
    m_points = std::move(x.m_points);

    return *this;
}

bool PointCloudDDS::operator ==(
        const PointCloudDDS& x) const
{

    return (m_seq == x.m_seq && m_points == x.m_points);
}

bool PointCloudDDS::operator !=(
        const PointCloudDDS& x) const
{
    return !(*this == x);
}

size_t PointCloudDDS::getMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    for(size_t a = 0; a < 10000; ++a)
    {
        current_alignment += PoseDDS::getMaxCdrSerializedSize(current_alignment);}


    return current_alignment - initial_alignment;
}

size_t PointCloudDDS::getCdrSerializedSize(
        const PointCloudDDS& data,
        size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    for(size_t a = 0; a < data.points().size(); ++a)
    {
        current_alignment += PoseDDS::getCdrSerializedSize(data.points().at(a), current_alignment);}


    return current_alignment - initial_alignment;
}

void PointCloudDDS::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{

    scdr << m_seq;
    scdr << m_points;

}

void PointCloudDDS::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{

    dcdr >> m_seq;
    dcdr >> m_points;
}

/*!
 * @brief This function sets a value in member seq
 * @param _seq New value for member seq
 */
void PointCloudDDS::seq(
        uint64_t _seq)
{
    m_seq = _seq;
}

/*!
 * @brief This function returns the value of member seq
 * @return Value of member seq
 */
uint64_t PointCloudDDS::seq() const
{
    return m_seq;
}

/*!
 * @brief This function returns a reference to member seq
 * @return Reference to member seq
 */
uint64_t& PointCloudDDS::seq()
{
    return m_seq;
}

/*!
 * @brief This function copies the value in member points
 * @param _points New value to be copied in member points
 */
void PointCloudDDS::points(
        const std::vector<PoseDDS>& _points)
{
    m_points = _points;
}

/*!
 * @brief This function moves the value in member points
 * @param _points New value to be moved in member points
 */
void PointCloudDDS::points(
        std::vector<PoseDDS>&& _points)
{
    m_points = std::move(_points);
}

/*!
 * @brief This function returns a constant reference to member points
 * @return Constant reference to member points
 */
const std::vector<PoseDDS>& PointCloudDDS::points() const
{
    return m_points;
}

/*!
 * @brief This function returns a reference to member points
 * @return Reference to member points
 */
std::vector<PoseDDS>& PointCloudDDS::points()
{
    return m_points;
}

size_t PointCloudDDS::getKeyMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t current_align = current_alignment;





    return current_align;
}

bool PointCloudDDS::isKeyDefined()
{
    return false;
}

void PointCloudDDS::serializeKey(
        eprosima::fastcdr::Cdr& scdr) const
{
    (void) scdr;
      
}

GeneralDataDDS::GeneralDataDDS()
{
    // m_seq com.eprosima.idl.parser.typecode.PrimitiveTypeCode@17d0685f
    m_seq = 0;
    // m_commandid com.eprosima.idl.parser.typecode.PrimitiveTypeCode@3891771e
    m_commandid = 0;
    // m_robotid com.eprosima.idl.parser.typecode.PrimitiveTypeCode@78ac1102
    m_robotid = 0;
    // m_ndparams com.eprosima.idl.parser.typecode.PrimitiveTypeCode@2de8284b
    m_ndparams = 0;
    // m_dparams com.eprosima.idl.parser.typecode.SequenceTypeCode@396e2f39

    // m_niparams com.eprosima.idl.parser.typecode.PrimitiveTypeCode@12c8a2c0
    m_niparams = 0;
    // m_iparams com.eprosima.idl.parser.typecode.SequenceTypeCode@7e0e6aa2

    // m_nbparams com.eprosima.idl.parser.typecode.PrimitiveTypeCode@18bf3d14
    m_nbparams = 0;
    // m_bparams com.eprosima.idl.parser.typecode.SequenceTypeCode@4fb64261

    // m_nsparams com.eprosima.idl.parser.typecode.PrimitiveTypeCode@1990a65e
    m_nsparams = 0;
    // m_sparams com.eprosima.idl.parser.typecode.SequenceTypeCode@64485a47


}

GeneralDataDDS::~GeneralDataDDS()
{











}

GeneralDataDDS::GeneralDataDDS(
        const GeneralDataDDS& x)
{
    m_seq = x.m_seq;
    m_commandid = x.m_commandid;
    m_robotid = x.m_robotid;
    m_ndparams = x.m_ndparams;
    m_dparams = x.m_dparams;
    m_niparams = x.m_niparams;
    m_iparams = x.m_iparams;
    m_nbparams = x.m_nbparams;
    m_bparams = x.m_bparams;
    m_nsparams = x.m_nsparams;
    m_sparams = x.m_sparams;
}

GeneralDataDDS::GeneralDataDDS(
        GeneralDataDDS&& x) noexcept 
{
    m_seq = x.m_seq;
    m_commandid = x.m_commandid;
    m_robotid = x.m_robotid;
    m_ndparams = x.m_ndparams;
    m_dparams = std::move(x.m_dparams);
    m_niparams = x.m_niparams;
    m_iparams = std::move(x.m_iparams);
    m_nbparams = x.m_nbparams;
    m_bparams = std::move(x.m_bparams);
    m_nsparams = x.m_nsparams;
    m_sparams = std::move(x.m_sparams);
}

GeneralDataDDS& GeneralDataDDS::operator =(
        const GeneralDataDDS& x)
{

    m_seq = x.m_seq;
    m_commandid = x.m_commandid;
    m_robotid = x.m_robotid;
    m_ndparams = x.m_ndparams;
    m_dparams = x.m_dparams;
    m_niparams = x.m_niparams;
    m_iparams = x.m_iparams;
    m_nbparams = x.m_nbparams;
    m_bparams = x.m_bparams;
    m_nsparams = x.m_nsparams;
    m_sparams = x.m_sparams;

    return *this;
}

GeneralDataDDS& GeneralDataDDS::operator =(
        GeneralDataDDS&& x) noexcept
{

    m_seq = x.m_seq;
    m_commandid = x.m_commandid;
    m_robotid = x.m_robotid;
    m_ndparams = x.m_ndparams;
    m_dparams = std::move(x.m_dparams);
    m_niparams = x.m_niparams;
    m_iparams = std::move(x.m_iparams);
    m_nbparams = x.m_nbparams;
    m_bparams = std::move(x.m_bparams);
    m_nsparams = x.m_nsparams;
    m_sparams = std::move(x.m_sparams);

    return *this;
}

bool GeneralDataDDS::operator ==(
        const GeneralDataDDS& x) const
{

    return (m_seq == x.m_seq && m_commandid == x.m_commandid && m_robotid == x.m_robotid && m_ndparams == x.m_ndparams && m_dparams == x.m_dparams && m_niparams == x.m_niparams && m_iparams == x.m_iparams && m_nbparams == x.m_nbparams && m_bparams == x.m_bparams && m_nsparams == x.m_nsparams && m_sparams == x.m_sparams);
}

bool GeneralDataDDS::operator !=(
        const GeneralDataDDS& x) const
{
    return !(*this == x);
}

size_t GeneralDataDDS::getMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += (100 * 8) + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);



    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += (100 * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += (100 * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    for(size_t a = 0; a < 10000; ++a)
    {
        current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4) + 255 + 1;
    }

    return current_alignment - initial_alignment;
}

size_t GeneralDataDDS::getCdrSerializedSize(
        const GeneralDataDDS& data,
        size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    if (data.dparams().size() > 0)
    {
        current_alignment += (data.dparams().size() * 8) + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);
    }



    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    if (data.iparams().size() > 0)
    {
        current_alignment += (data.iparams().size() * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);
    }



    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    if (data.bparams().size() > 0)
    {
        current_alignment += (data.bparams().size() * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);
    }



    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    for(size_t a = 0; a < data.sparams().size(); ++a)
    {
        current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4) +
            data.sparams().at(a).size() + 1;
    }

    return current_alignment - initial_alignment;
}

void GeneralDataDDS::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{

    scdr << m_seq;
    scdr << m_commandid;
    scdr << m_robotid;
    scdr << m_ndparams;
    scdr << m_dparams;
    scdr << m_niparams;
    scdr << m_iparams;
    scdr << m_nbparams;
    scdr << m_bparams;
    scdr << m_nsparams;
    scdr << m_sparams;

}

void GeneralDataDDS::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{

    dcdr >> m_seq;
    dcdr >> m_commandid;
    dcdr >> m_robotid;
    dcdr >> m_ndparams;
    dcdr >> m_dparams;
    dcdr >> m_niparams;
    dcdr >> m_iparams;
    dcdr >> m_nbparams;
    dcdr >> m_bparams;
    dcdr >> m_nsparams;
    dcdr >> m_sparams;
}

/*!
 * @brief This function sets a value in member seq
 * @param _seq New value for member seq
 */
void GeneralDataDDS::seq(
        uint64_t _seq)
{
    m_seq = _seq;
}

/*!
 * @brief This function returns the value of member seq
 * @return Value of member seq
 */
uint64_t GeneralDataDDS::seq() const
{
    return m_seq;
}

/*!
 * @brief This function returns a reference to member seq
 * @return Reference to member seq
 */
uint64_t& GeneralDataDDS::seq()
{
    return m_seq;
}

/*!
 * @brief This function sets a value in member commandid
 * @param _commandid New value for member commandid
 */
void GeneralDataDDS::commandid(
        char _commandid)
{
    m_commandid = _commandid;
}

/*!
 * @brief This function returns the value of member commandid
 * @return Value of member commandid
 */
char GeneralDataDDS::commandid() const
{
    return m_commandid;
}

/*!
 * @brief This function returns a reference to member commandid
 * @return Reference to member commandid
 */
char& GeneralDataDDS::commandid()
{
    return m_commandid;
}

/*!
 * @brief This function sets a value in member robotid
 * @param _robotid New value for member robotid
 */
void GeneralDataDDS::robotid(
        char _robotid)
{
    m_robotid = _robotid;
}

/*!
 * @brief This function returns the value of member robotid
 * @return Value of member robotid
 */
char GeneralDataDDS::robotid() const
{
    return m_robotid;
}

/*!
 * @brief This function returns a reference to member robotid
 * @return Reference to member robotid
 */
char& GeneralDataDDS::robotid()
{
    return m_robotid;
}

/*!
 * @brief This function sets a value in member ndparams
 * @param _ndparams New value for member ndparams
 */
void GeneralDataDDS::ndparams(
        char _ndparams)
{
    m_ndparams = _ndparams;
}

/*!
 * @brief This function returns the value of member ndparams
 * @return Value of member ndparams
 */
char GeneralDataDDS::ndparams() const
{
    return m_ndparams;
}

/*!
 * @brief This function returns a reference to member ndparams
 * @return Reference to member ndparams
 */
char& GeneralDataDDS::ndparams()
{
    return m_ndparams;
}

/*!
 * @brief This function copies the value in member dparams
 * @param _dparams New value to be copied in member dparams
 */
void GeneralDataDDS::dparams(
        const std::vector<double>& _dparams)
{
    m_dparams = _dparams;
}

/*!
 * @brief This function moves the value in member dparams
 * @param _dparams New value to be moved in member dparams
 */
void GeneralDataDDS::dparams(
        std::vector<double>&& _dparams)
{
    m_dparams = std::move(_dparams);
}

/*!
 * @brief This function returns a constant reference to member dparams
 * @return Constant reference to member dparams
 */
const std::vector<double>& GeneralDataDDS::dparams() const
{
    return m_dparams;
}

/*!
 * @brief This function returns a reference to member dparams
 * @return Reference to member dparams
 */
std::vector<double>& GeneralDataDDS::dparams()
{
    return m_dparams;
}
/*!
 * @brief This function sets a value in member niparams
 * @param _niparams New value for member niparams
 */
void GeneralDataDDS::niparams(
        char _niparams)
{
    m_niparams = _niparams;
}

/*!
 * @brief This function returns the value of member niparams
 * @return Value of member niparams
 */
char GeneralDataDDS::niparams() const
{
    return m_niparams;
}

/*!
 * @brief This function returns a reference to member niparams
 * @return Reference to member niparams
 */
char& GeneralDataDDS::niparams()
{
    return m_niparams;
}

/*!
 * @brief This function copies the value in member iparams
 * @param _iparams New value to be copied in member iparams
 */
void GeneralDataDDS::iparams(
        const std::vector<char>& _iparams)
{
    m_iparams = _iparams;
}

/*!
 * @brief This function moves the value in member iparams
 * @param _iparams New value to be moved in member iparams
 */
void GeneralDataDDS::iparams(
        std::vector<char>&& _iparams)
{
    m_iparams = std::move(_iparams);
}

/*!
 * @brief This function returns a constant reference to member iparams
 * @return Constant reference to member iparams
 */
const std::vector<char>& GeneralDataDDS::iparams() const
{
    return m_iparams;
}

/*!
 * @brief This function returns a reference to member iparams
 * @return Reference to member iparams
 */
std::vector<char>& GeneralDataDDS::iparams()
{
    return m_iparams;
}
/*!
 * @brief This function sets a value in member nbparams
 * @param _nbparams New value for member nbparams
 */
void GeneralDataDDS::nbparams(
        char _nbparams)
{
    m_nbparams = _nbparams;
}

/*!
 * @brief This function returns the value of member nbparams
 * @return Value of member nbparams
 */
char GeneralDataDDS::nbparams() const
{
    return m_nbparams;
}

/*!
 * @brief This function returns a reference to member nbparams
 * @return Reference to member nbparams
 */
char& GeneralDataDDS::nbparams()
{
    return m_nbparams;
}

/*!
 * @brief This function copies the value in member bparams
 * @param _bparams New value to be copied in member bparams
 */
void GeneralDataDDS::bparams(
        const std::vector<uint8_t>& _bparams)
{
    m_bparams = _bparams;
}

/*!
 * @brief This function moves the value in member bparams
 * @param _bparams New value to be moved in member bparams
 */
void GeneralDataDDS::bparams(
        std::vector<uint8_t>&& _bparams)
{
    m_bparams = std::move(_bparams);
}

/*!
 * @brief This function returns a constant reference to member bparams
 * @return Constant reference to member bparams
 */
const std::vector<uint8_t>& GeneralDataDDS::bparams() const
{
    return m_bparams;
}

/*!
 * @brief This function returns a reference to member bparams
 * @return Reference to member bparams
 */
std::vector<uint8_t>& GeneralDataDDS::bparams()
{
    return m_bparams;
}
/*!
 * @brief This function sets a value in member nsparams
 * @param _nsparams New value for member nsparams
 */
void GeneralDataDDS::nsparams(
        char _nsparams)
{
    m_nsparams = _nsparams;
}

/*!
 * @brief This function returns the value of member nsparams
 * @return Value of member nsparams
 */
char GeneralDataDDS::nsparams() const
{
    return m_nsparams;
}

/*!
 * @brief This function returns a reference to member nsparams
 * @return Reference to member nsparams
 */
char& GeneralDataDDS::nsparams()
{
    return m_nsparams;
}

/*!
 * @brief This function copies the value in member sparams
 * @param _sparams New value to be copied in member sparams
 */
void GeneralDataDDS::sparams(
        const std::vector<std::string>& _sparams)
{
    m_sparams = _sparams;
}

/*!
 * @brief This function moves the value in member sparams
 * @param _sparams New value to be moved in member sparams
 */
void GeneralDataDDS::sparams(
        std::vector<std::string>&& _sparams)
{
    m_sparams = std::move(_sparams);
}

/*!
 * @brief This function returns a constant reference to member sparams
 * @return Constant reference to member sparams
 */
const std::vector<std::string>& GeneralDataDDS::sparams() const
{
    return m_sparams;
}

/*!
 * @brief This function returns a reference to member sparams
 * @return Reference to member sparams
 */
std::vector<std::string>& GeneralDataDDS::sparams()
{
    return m_sparams;
}

size_t GeneralDataDDS::getKeyMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t current_align = current_alignment;














    return current_align;
}

bool GeneralDataDDS::isKeyDefined()
{
    return false;
}

void GeneralDataDDS::serializeKey(
        eprosima::fastcdr::Cdr& scdr) const
{
    (void) scdr;
               
}

NAVPointCloudDDS::NAVPointCloudDDS()
{
    // m_points com.eprosima.fastdds.idl.parser.typecode.StructTypeCode@401e7803

    // m_pose com.eprosima.fastdds.idl.parser.typecode.StructTypeCode@10dba097


}

NAVPointCloudDDS::~NAVPointCloudDDS()
{


}

NAVPointCloudDDS::NAVPointCloudDDS(
        const NAVPointCloudDDS& x)
{
    m_points = x.m_points;
    m_pose = x.m_pose;
}

NAVPointCloudDDS::NAVPointCloudDDS(
        NAVPointCloudDDS&& x) noexcept 
{
    m_points = std::move(x.m_points);
    m_pose = std::move(x.m_pose);
}

NAVPointCloudDDS& NAVPointCloudDDS::operator =(
        const NAVPointCloudDDS& x)
{

    m_points = x.m_points;
    m_pose = x.m_pose;

    return *this;
}

NAVPointCloudDDS& NAVPointCloudDDS::operator =(
        NAVPointCloudDDS&& x) noexcept
{

    m_points = std::move(x.m_points);
    m_pose = std::move(x.m_pose);

    return *this;
}

bool NAVPointCloudDDS::operator ==(
        const NAVPointCloudDDS& x) const
{

    return (m_points == x.m_points && m_pose == x.m_pose);
}

bool NAVPointCloudDDS::operator !=(
        const NAVPointCloudDDS& x) const
{
    return !(*this == x);
}

size_t NAVPointCloudDDS::getMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += PointCloudDDS::getMaxCdrSerializedSize(current_alignment);
    current_alignment += PoseDDS::getMaxCdrSerializedSize(current_alignment);

    return current_alignment - initial_alignment;
}

size_t NAVPointCloudDDS::getCdrSerializedSize(
        const NAVPointCloudDDS& data,
        size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += PointCloudDDS::getCdrSerializedSize(data.points(), current_alignment);
    current_alignment += PoseDDS::getCdrSerializedSize(data.pose(), current_alignment);

    return current_alignment - initial_alignment;
}

void NAVPointCloudDDS::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{

    scdr << m_points;
    scdr << m_pose;

}

void NAVPointCloudDDS::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{

    dcdr >> m_points;
    dcdr >> m_pose;
}

/*!
 * @brief This function copies the value in member points
 * @param _points New value to be copied in member points
 */
void NAVPointCloudDDS::points(
        const PointCloudDDS& _points)
{
    m_points = _points;
}

/*!
 * @brief This function moves the value in member points
 * @param _points New value to be moved in member points
 */
void NAVPointCloudDDS::points(
        PointCloudDDS&& _points)
{
    m_points = std::move(_points);
}

/*!
 * @brief This function returns a constant reference to member points
 * @return Constant reference to member points
 */
const PointCloudDDS& NAVPointCloudDDS::points() const
{
    return m_points;
}

/*!
 * @brief This function returns a reference to member points
 * @return Reference to member points
 */
PointCloudDDS& NAVPointCloudDDS::points()
{
    return m_points;
}
/*!
 * @brief This function copies the value in member pose
 * @param _pose New value to be copied in member pose
 */
void NAVPointCloudDDS::pose(
        const PoseDDS& _pose)
{
    m_pose = _pose;
}

/*!
 * @brief This function moves the value in member pose
 * @param _pose New value to be moved in member pose
 */
void NAVPointCloudDDS::pose(
        PoseDDS&& _pose)
{
    m_pose = std::move(_pose);
}

/*!
 * @brief This function returns a constant reference to member pose
 * @return Constant reference to member pose
 */
const PoseDDS& NAVPointCloudDDS::pose() const
{
    return m_pose;
}

/*!
 * @brief This function returns a reference to member pose
 * @return Reference to member pose
 */
PoseDDS& NAVPointCloudDDS::pose()
{
    return m_pose;
}

size_t NAVPointCloudDDS::getKeyMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t current_align = current_alignment;





    return current_align;
}

bool NAVPointCloudDDS::isKeyDefined()
{
    return false;
}

void NAVPointCloudDDS::serializeKey(
        eprosima::fastcdr::Cdr& scdr) const
{
    (void) scdr;
      
}
