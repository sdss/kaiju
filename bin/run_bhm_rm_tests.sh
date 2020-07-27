#!/usr/bin/env bash


DATEVERSION='20200727v6'

OUTDIR="${HOME}/scratch/kaiju2"
mkdir -p $OUTDIR
cd $OUTDIR

read -r -d '' SQL << EOM
drop table sandbox."td_test_XMM-LSS";

CREATE TABLE sandbox."td_test_XMM-LSS" AS SELECT
    DISTINCT ON(t.catalogid)
    t.catalogid, ROW_NUMBER() OVER (ORDER BY t.catalogid) as targetid, t.catalogid as pk,
    t.ra, t.dec,
    COUNT(*) OVER (PARTITION BY t.catalogid) as ncartons, string_agg(c.carton,',') OVER (PARTITION BY t.catalogid) as cartons,
    c2t.priority AS priority,
    1000 as value, 'bhm_rm_178x8' as cadence,
    m.g, m.r, m.i, m.z, m.bp, m.rp,
    r.mi as rm_mi, r.mg as rm_mg
FROM target AS t
JOIN carton_to_target AS c2t
    ON t.pk = c2t.target_pk
JOIN carton AS c
    ON c2t.carton_pk = c.pk
JOIN catalog_to_bhm_rm_v0 as c2r
    ON t.catalogid = c2r.catalogid
JOIN bhm_rm_v0 as r
    ON c2r.target_id = r.pk
LEFT OUTER JOIN magnitude AS m
    ON m.carton_to_target_pk = c2t.pk
WHERE
    c.carton ~ '^bhm_rm_' AND
    q3c_radial_query(t.ra,t.dec,35.70833,-5.05000,1.49) AND c.version_pk = 49
ORDER BY t.catalogid,c2t.priority ASC;


drop table sandbox."td_test_COSMOS";
CREATE TABLE sandbox."td_test_COSMOS" AS SELECT
    DISTINCT ON(t.catalogid)
    t.catalogid, ROW_NUMBER() OVER (ORDER BY t.catalogid) as targetid, t.catalogid as pk,
    t.ra, t.dec,
    COUNT(*) OVER (PARTITION BY t.catalogid) as ncartons, string_agg(c.carton,',') OVER (PARTITION BY t.catalogid) as cartons,
    c2t.priority AS priority,
    1000 as value, 'bhm_rm_178x8' as cadence,
    m.g, m.r, m.i, m.z, m.bp, m.rp,
    r.mi as rm_mi, r.mg as rm_mg
FROM target AS t
JOIN carton_to_target AS c2t
    ON t.pk = c2t.target_pk
JOIN carton AS c
    ON c2t.carton_pk = c.pk
JOIN catalog_to_bhm_rm_v0 as c2r
    ON t.catalogid = c2r.catalogid
JOIN bhm_rm_v0 as r
    ON c2r.target_id = r.pk
LEFT OUTER JOIN magnitude AS m
    ON m.carton_to_target_pk = c2t.pk
WHERE
    c.carton ~ '^bhm_rm_' AND
    q3c_radial_query(t.ra,t.dec,150.0,2.2,1.49) AND c.version_pk = 49
ORDER BY t.catalogid,c2t.priority ASC;



drop table sandbox."td_test_SDSS-RM";
CREATE TABLE sandbox."td_test_SDSS-RM" AS SELECT
    DISTINCT ON(t.catalogid)
    t.catalogid, ROW_NUMBER() OVER (ORDER BY t.catalogid) as targetid, t.catalogid as pk,
    t.ra, t.dec,
    COUNT(*) OVER (PARTITION BY t.catalogid) as ncartons, string_agg(c.carton,',') OVER (PARTITION BY t.catalogid) as cartons,
    c2t.priority AS priority,
    1000 as value, 'bhm_rm_178x8' as cadence,
    m.g, m.r, m.i, m.z, m.bp, m.rp,
    r.mi as rm_mi, r.mg as rm_mg
FROM target AS t
JOIN carton_to_target AS c2t
    ON t.pk = c2t.target_pk
JOIN carton AS c
    ON c2t.carton_pk = c.pk
JOIN catalog_to_bhm_rm_v0 as c2r
    ON t.catalogid = c2r.catalogid
JOIN bhm_rm_v0 as r
    ON c2r.target_id = r.pk
LEFT OUTER JOIN magnitude AS m
    ON m.carton_to_target_pk = c2t.pk
WHERE
    c.carton ~ '^bhm_rm_' AND
    q3c_radial_query(t.ra,t.dec,213.7042,53.08333,1.49) AND c.version_pk = 49
ORDER BY t.catalogid,c2t.priority ASC;
EOM

echo "$SQL"

psql -d sdss5db -U sdss_user -h localhost -p 7502 -c "$SQL"


psql -d sdss5db -U sdss_user -h localhost -p 7502 -c $'\\copy (SELECT * FROM sandbox."td_test_XMM-LSS") TO 'td_test_XMM-LSS.csv' with csv header';
psql -d sdss5db -U sdss_user -h localhost -p 7502 -c $'\\copy (SELECT * FROM sandbox."td_test_COSMOS")  TO 'td_test_COSMOS.csv'  with csv header';
psql -d sdss5db -U sdss_user -h localhost -p 7502 -c $'\\copy (SELECT * FROM sandbox."td_test_SDSS-RM") TO 'td_test_SDSS-RM.csv' with csv header';


for F in td_test_*.csv; do
  stilts tpipe in=${F} out="${F%.*}.fits" ifmt=csv ofmt=fits-basic
done
num_rows td_test*fits

## run the kaiju stuff

~/SDSSV/gitwork/kaiju/bin/bhm_rm_tests -b 3.0 -n 100 | tee master.log



gawk '$4~/STATS/ {n[$2]++; sum[$2]+=$6; nhex[$2]=$8; if($6>nmax[$2]){nmax[$2]=$6;best_iter[$2]=$3}; if ($6<nmin[$2]||nmin[$2]==0){nmin[$2]=$6;}}; END {printf("#%7s %8s %8s %6s %6s %6s %6s\n", "FIELD", "nruns", "bestRun", "Nbest", "Nworst", "Nmean", "Nhex" ); for (f in n){printf("%8s %8d %8s %6d %6d %6.1f %6s\n", f,n[f],best_iter[f],nmax[f],nmin[f],sum[f]/n[f], nhex[f])}} ' master.log  | tee  master_field_stats.txt


for F in "XMM-LSS" "COSMOS" "SDSS-RM" ; do
   # choose the best run
   I=`gawk -v f="$F" '$1==f {print $3}' master_field_stats.txt`
   ln -sf kaiju_assignments_${F}_${I}.fits kaiju_assignments_${F}_best.fits
   ftcopy "kaiju_assignments_${F}_best.fits[1][assigned][col catalogid(K)=pk,ra,dec,assigned,rmfield(8A)='${F}']" kaiju_assignable_BHM-RM_targets_${F}.fits clobber=yes mode=q copyall=no
done
ftmerge kaiju_assignable_BHM-RM_targets_XMM-LSS.fits,kaiju_assignable_BHM-RM_targets_COSMOS.fits,kaiju_assignable_BHM-RM_targets_SDSS-RM.fits kaiju_assignable_BHM-RM_targets.fits columns=-assigned clobber=yes mode=q

qfstat kaiju_assignable_BHM-RM_targets.fits rmfield

tar -cvzhf rm_best_kaiju_runs_${DATEVERSION}.tar.gz master_field_stats.txt master.log td_test_*.fits kaiju_assignments_*_best.fits
