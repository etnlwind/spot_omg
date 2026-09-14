# S side comparison

Both images use the same side camera, 2.754 kg model and D37.3 x 27 mm cushions after 2 seconds of physical settling. Top/before: original [0,45,90] deg. Bottom/after: contact-aligned S. Contact means centroid of lowest rigid cushion mesh vertices; R2 reference is the CAD j2 joint axis anchor, not an independently measured motor housing center. Nominal body-frame X errors: before 3.060 mm, after below 0.001 mm. Under load, finite servo stiffness and body tilt can produce additional deviation. These are simulator results, not hardware validation. Shared firmware gait preparation is unchanged.

See measurements.json for angles, nominal errors and settled tilt. Default-model and virtual-controller regression: 33 passed; existing numerical matrix warnings remain, with model geometry explicitly checked for finite values.
