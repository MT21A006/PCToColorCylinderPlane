#ifndef PCL_FILTERS_CROP_PLANE_H_
#define PCL_FILTERS_CROP_PLANE_H_

#include <pcl/point_types.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>

namespace pcl
{
    /** \brief CropPlane is a filter that allows the user to filter all the data
      * in one side of a plane.
      *
      * \author Yasuhiro Masutani
      * \ingroup filters
      */
    template<typename PointT>
    class CropPlane : public FilterIndices<PointT>
    {
        using Filter<PointT>::getClassName;

        typedef typename Filter<PointT>::PointCloud PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:

        typedef pcl::shared_ptr< CropPlane<PointT> > Ptr;
        typedef pcl::shared_ptr< const CropPlane<PointT> > ConstPtr;

        /** \brief Constructor.
          * \param[in] extract_removed_indices Set to true if you want to be able to extract the indices of points being removed (default = false).
          */
        CropPlane(bool extract_removed_indices = false) :
            FilterIndices<PointT>::FilterIndices(extract_removed_indices),
            plane_params_(Eigen::Vector4f(1, 0, 0, 0)),
            offset_(0)
        {
            filter_name_ = "CropPlane";
        }

        /** \brief Set the parameters of the plane
          * \param[in] plane_params the parameters of the plane
          */
        inline void
            setPlane(const Eigen::Vector4f& plane_params)
        {
            Eigen::Vector3f p3 = plane_params.head(3);
            plane_params_ = plane_params / p3.norm();
        }

        /** \brief Get the value of the parameters of the plane, as set by the user
          * \return the value of the internal \a plane_params parameter.
          */
        inline Eigen::Vector4f
            getPlane() const
        {
            return (plane_params_);
        }

        /** \brief Set the offset from the plane
          * \param[in] distance the offset from the plane
          */
        inline void
            setOffset(float offset)
        {
            offset_ = offset;
        }

        /** \brief Get the value of the offset from the plane as set by the user
          * \return the value of the internal \a offset parameter.
          */
        inline float
            getOffset() const
        {
            return (offset_);
        }

    protected:
        using PCLBase<PointT>::input_;
        using PCLBase<PointT>::indices_;
        using Filter<PointT>::filter_name_;
        using FilterIndices<PointT>::negative_;
        using FilterIndices<PointT>::keep_organized_;
        using FilterIndices<PointT>::user_filter_value_;
        using FilterIndices<PointT>::extract_removed_indices_;
        using FilterIndices<PointT>::removed_indices_;

        /** \brief Sample of point indices into a separate PointCloud
          * \param[out] output the resultant point cloud
          */
        void
            applyFilter(PointCloud& output);

        /** \brief Sample of point indices
          * \param[out] indices the resultant point cloud indices
          */
        void
            applyFilter(std::vector<int>& indices);

    private:
        /** \brief The plane parameters. */
        Eigen::Vector4f plane_params_;
        /** \brief The offset from the plane. */
        float offset_;
    };

}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::CropPlane<PointT>::applyFilter(PointCloud& output)
{
    std::vector<int> indices;
    if (keep_organized_)
    {
        bool temp = extract_removed_indices_;
        extract_removed_indices_ = true;
        applyFilter(indices);
        extract_removed_indices_ = temp;

        output = *input_;
        for (int rii = 0; rii < static_cast<int> (removed_indices_->size()); ++rii)  // rii = removed indices iterator
            output.points[(*removed_indices_)[rii]].x = output.points[(*removed_indices_)[rii]].y = output.points[(*removed_indices_)[rii]].z = user_filter_value_;
        if (!pcl_isfinite(user_filter_value_))
            output.is_dense = false;
    }
    else
    {
        output.is_dense = true;
        applyFilter(indices);
        pcl::copyPointCloud(*input_, indices, output);
    }
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::CropPlane<PointT>::applyFilter(std::vector<int>& indices)
{
    indices.resize(input_->points.size());
    removed_indices_->resize(input_->points.size());
    int indices_count = 0;
    int removed_indices_count = 0;
    for (size_t index = 0; index < indices_->size(); ++index)
    {
        if (!input_->is_dense)
            // Check if the point is invalid
            if (!isFinite(input_->points[index]))
                continue;

        // Get local point
        PointT pt = input_->points[(*indices_)[index]];

        // If negative side of the plane, remove
        if (pt.x * plane_params_[0] + pt.y * plane_params_[1] + pt.z * plane_params_[2] + plane_params_[3] < offset_)
        {
            if (negative_)
                indices[indices_count++] = (*indices_)[index];
            else if (extract_removed_indices_)
                (*removed_indices_)[removed_indices_count++] = static_cast<int> (index);
        }
        // If near from the plane
        else
        {
            if (negative_ && extract_removed_indices_)
                (*removed_indices_)[removed_indices_count++] = static_cast<int> (index);
            else if (!negative_)
                indices[indices_count++] = (*indices_)[index];
        }
    }
    indices.resize(indices_count);
    removed_indices_->resize(removed_indices_count);
}

#define PCL_INSTANTIATE_CROPPLANE(T) template class PCL_EXPORTS pcl::CropPlane<T>;

#endif  // PCL_FILTERS_CROP_PLANE_H_