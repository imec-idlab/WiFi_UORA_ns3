/*
 * Copyright (c) 2024 IDLab ( UAntwerpen & IMEC)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Douglas D. Agbeve <douglas.agbeve@uantwerpen.be>
 */

#include "uora-parameter-set.h"

#include <cmath>

namespace ns3
{

UoraParameterSet::UoraParameterSet()
    : m_ocwRange(0)
{
}

WifiInformationElementId
UoraParameterSet::ElementId() const
{
    return IE_EXTENSION;
}

WifiInformationElementId
UoraParameterSet::ElementIdExt() const
{
    return IE_EXT_UORA_PARAMETER_SET;
}


void
UoraParameterSet::SetOCwMin(uint8_t OCwMin)
{
    NS_ABORT_MSG_IF(OCwMin > 127, "CWmin exceeds the maximum value");

    auto eOCwMin = std::log2(OCwMin + 1);
    NS_ABORT_MSG_IF(std::trunc(eOCwMin) != eOCwMin, "CWmin is not a power of 2 minus 1");

    m_ocwRange |= (static_cast<uint8_t>(eOCwMin) & 0x07);
}

void
UoraParameterSet::SetOCwMax(uint8_t OCwMax)
{
    NS_ABORT_MSG_IF(OCwMax > 127, "CWmin exceeds the maximum value");

    auto eOCwMax = std::log2(OCwMax + 1);
    NS_ABORT_MSG_IF(std::trunc(eOCwMax) != eOCwMax, "CWmax is not a power of 2 minus 1");

    m_ocwRange |= (static_cast<uint8_t>(eOCwMax) & 0x07) << 3;
}


uint8_t
UoraParameterSet::GetOCwMin() const
{
    uint8_t eOCwMin = (m_ocwRange & 0x07);
    return static_cast<uint8_t>(std::exp2(eOCwMin) - 1);
}

uint8_t
UoraParameterSet::GetOCwMax() const
{
    uint8_t eOCwMax = ((m_ocwRange >> 3) & 0x07);
    return static_cast<uint8_t>(std::exp2(eOCwMax) - 1);
}


uint16_t
UoraParameterSet::GetInformationFieldSize() const
{
    // ElementIdExt (1)  + OCW Range
    return 2;
}

void
UoraParameterSet::SerializeInformationField(Buffer::Iterator start) const
{
    start.WriteU8(m_ocwRange);
}

uint16_t
UoraParameterSet::DeserializeInformationField(Buffer::Iterator start, uint16_t length)
{
    Buffer::Iterator i = start;
    m_ocwRange = i.ReadU8();
    return 1;
}

} // namespace ns3
