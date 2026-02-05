package fr.intuite.rtlsdrbridge

import com.google.gson.annotations.SerializedName

data class DeviceCategory(
    @SerializedName("vendorId") val vendorId: String,
    @SerializedName("productId") val productId: String,
    @SerializedName("name") val name: String,
    @SerializedName("driver") val driver: String
)
