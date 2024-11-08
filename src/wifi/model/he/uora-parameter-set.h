/*
 * Copyright (c) 2024 IDLab ( University of Antwerp & IMEC)
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

#ifndef UORA_PARAMETER_SET_H
#define UORA_PARAMETER_SET_H

#include "ns3/wifi-information-element.h"

namespace ns3
{

/**
 * \brief The UORA Parameter Set
 * \ingroup wifi
 *
 * The 802.11ax UORA Parameter Set.
 */
class UoraParameterSet : public WifiInformationElement
{
  public:
    UoraParameterSet();

    WifiInformationElementId ElementId() const override;
    WifiInformationElementId ElementIdExt() const override;

    /**
     * Set the EOCWmin subfield of the ECWmin/ECWmax field in the MU AC Parameter Record
     * field corresponding to the given AC Index (<i>aci</i>). Note that <i>cwMin</i>
     * must be a power of 2 minus 1 in the range from 0 to 32767.
     *
     * \param aci the AC Index
     * \param cwMin the CWmin value encoded by the ECWmin field
     */
    void SetOCwMin(uint8_t cwMin);
    /**
     * Set the ECWmax subfield of the ECWmin/ECWmax field in the MU AC Parameter Record
     * field corresponding to the given AC Index (<i>aci</i>). Note that <i>cwMax</i>
     * must be a power of 2 minus 1 in the range from 0 to 32767.
     *
     * \param aci the AC Index
     * \param cwMax the CWmax value encoded by the ECWmax field
     */
    void SetOCwMax(uint8_t cwMax);
    /**
     * Get the CWmin value encoded by the ECWmin subfield of the ECWmin/ECWmax field
     * in the MU AC Parameter Record field corresponding to the given AC Index (<i>aci</i>).
     *
     * \param aci the AC Index
     * \return the CWmin value
     */
    uint8_t GetOCwMin() const;
    /**
     * Get the CWmax value encoded by the ECWmax subfield of the ECWmin/ECWmax field
     * in the MU AC Parameter Record field corresponding to the given AC Index (<i>aci</i>).
     *
     * \param aci the AC Index
     * \return the CWmax value
     */
    uint8_t GetOCwMax() const;

  private:
    uint16_t GetInformationFieldSize() const override;
    void SerializeInformationField(Buffer::Iterator start) const override;
    uint16_t DeserializeInformationField(Buffer::Iterator start, uint16_t length) override;


    uint8_t m_ocwRange; ///< OCW Range field
};

} // namespace ns3

#endif /* UORA_PARAMETER_SET_H */
